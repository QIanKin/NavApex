#include "planner.hpp"
#include "planner/path_plan/searcher_io.hpp"
#include "robot_model/robot_state.hpp"

#include <perception/esdf/esdf_map_2d.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <tools/log.hpp>
#include <utility>

namespace navapex::planner
{
namespace
{
constexpr char PLANNER_TAG[] = "planner";
constexpr double PI = 3.14159265358979323846;

double wrap(double angle) { return std::remainder(angle, 2 * PI); }
double unwrap(double angle, double reference) { return reference + wrap(angle - reference); }
double elapsedMs(std::chrono::steady_clock::time_point begin)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

bool lineFree(const perception::OccupancyMap &map, const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    int count = std::max(2, static_cast<int>(std::ceil((b - a).head<2>().norm() / map.resolution() * 2)));
    for (int i = 0; i <= count; ++i)
    {
        Eigen::Vector3d position = a + (b - a) * (static_cast<double>(i) / count);
        if (map.occupied({position.x(), position.y(), position.z()}))
            return false;
    }
    return true;
}
} // namespace



Planner::Planner(std::shared_ptr<const robot_model::RobotModel> robot) 
                 : robot_(std::move(robot))
{
    auto path = std::filesystem::path(__FILE__).parent_path() / "config" / "planner.json";
    param_ = PlannerParams(path.string());
    if (param_.type == "diff") {
        search_input_ = std::make_shared<path_plan::SearchInput>();
        searcher_ = std::make_unique<path_plan::SearcherInterface>("astar2d", search_input_);
        optimizer_input_ = std::make_shared<traj_opt::OptimizerInput>();
        optimizer_ = std::make_unique<traj_opt::OptimizerInterface>(param_.type, optimizer_input_);
        optimizer_input_->robot_ = robot_;
    }
}

void Planner::discardWarmStart()
{
    previous_trajectory_ = {};
    force_search_ = true;
}

const PlannerParams &Planner::param() const
{ 
    return param_; 
}

const std::vector<Eigen::Vector3d> &Planner::globalPath() const 
{ 
    return global_path_;
}

void Planner::reset()
{
    phase_ = MotionPhase::NewPath;
    global_path_.clear();
    path_lengths_.clear();
    previous_trajectory_ = {};
    force_search_   = false;
    rotation_ready_ = false;
    alignment_yaw_  = 0;
    stopped_since_  = -1;
    progress_ = 0;
}

GlobalPlanResult Planner::planGlobal(const GlobalPlanInput &input)
{
    auto begin = std::chrono::steady_clock::now();
    reset();

    GlobalPlanResult result;
    search_input_->start = toPosition3D(input.start);
    search_input_->goal  = toPosition3D(input.goal);
    search_input_->map   = input.map; 
    search_input_->radius = robot_->radius();
    auto search_result = searcher_->search();
    if (!search_result.success)
    {
        LOGW(PLANNER_TAG) << "global search failed, elapsed_ms=" << elapsedMs(begin);
        return result;
    }

    global_path_ = std::move(search_result.path);
    goal_ = Eigen::Vector3d(input.goal.x,input.goal.y,input.goal.yaw);
    path_lengths_.resize(global_path_.size());
    path_lengths_[0] = 0;
    for (std::size_t i = 1; i < global_path_.size(); ++i)
    {
        path_lengths_[i] = path_lengths_[i - 1] + (global_path_[i] - global_path_[i - 1]).head<2>().norm();
    }

    result.path = global_path_;
    result.success = true;
    LOGI(PLANNER_TAG) << "global path points=" << result.path.size() << " length=" << path_lengths_.back()
                      << " elapsed_ms=" << elapsedMs(begin);
    return result;
}

double Planner::updateProgress(const Eigen::Vector2d &position)
{
    double best = std::numeric_limits<double>::infinity(), selected = progress_;
    for (std::size_t i = 1; i < global_path_.size(); ++i)
    {
        if (path_lengths_[i] < progress_ || path_lengths_[i - 1] > progress_ + 2 * param_.lookahead_distance)
            continue;
        Eigen::Vector2d a = global_path_[i - 1].head<2>(), edge = global_path_[i].head<2>() - a;
        double length = edge.norm();
        if (length < 1e-9) continue;
        double u = std::clamp((position - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
        double distance = (position - a - u * edge).squaredNorm();
        if (distance < best)
        {
            best = distance;
            selected = path_lengths_[i - 1] + u * length;
        }
    }
    progress_ = std::max(progress_, selected);
    return progress_;
}

Eigen::Vector3d Planner::pathPoint(double distance) const
{
    if (global_path_.size() == 1 || distance >= path_lengths_.back())
        return global_path_.back();
    auto next = std::upper_bound(path_lengths_.begin(), path_lengths_.end(), distance);
    std::size_t i = std::max<std::size_t>(1, next - path_lengths_.begin());
    double length = path_lengths_[i] - path_lengths_[i - 1];
    double u = length > 1e-9 ? (distance - path_lengths_[i - 1]) / length : 0;
    return global_path_[i - 1] + u * (global_path_[i] - global_path_[i - 1]);
}

double Planner::pathYaw(double distance, double fallback_yaw) const
{
    double window = param_.waypoint_spacing;
    double lower = std::max(0.0, std::min(distance - window / 2, path_lengths_.back() - window));
    Eigen::Vector2d delta = (pathPoint(std::min(path_lengths_.back(), lower + window)) - pathPoint(lower)).head<2>();
    return delta.norm() > 1e-9 ? std::atan2(delta.y(), delta.x()) : fallback_yaw;
}

LocalPlanResult Planner::planLocal(const LocalPlanInput &input)
{
    auto begin = std::chrono::steady_clock::now();
    double esdf_ms = 0;
    const char *initial_source = "";
    const char *reset_reason = "";
    LocalPlanResult result;
    auto fail = [&](const char *reason)
    {
        stopped_since_ = -1;
        discardWarmStart();
        LOGW(PLANNER_TAG) << "local plan failed reason=" << reason << " seed=" << initial_source
                        << " reset=" << reset_reason << " esdf_ms=" << esdf_ms << " elapsed_ms=" << elapsedMs(begin);
        return result;
    };
    if (global_path_.empty()) return fail("no_global_path");
    const auto &state = input.state;
    auto planar_pose = toPose2D(state.pose);
    Eigen::Vector3d pose(planar_pose.x, planar_pose.y, planar_pose.yaw);
    double linear_velocity = state.twist.linear.x;
    double angular_velocity = state.twist.angular.z;
    const auto &linear_acceleration = state.acceleration.linear;
    double angular_acceleration = state.acceleration.angular.z;
    const auto &control = param_;
    if (!input.map || input.map->space() != perception::Space::Planar || !pose.allFinite() || !std::isfinite(linear_velocity) ||
        !std::isfinite(angular_velocity) || !std::isfinite(linear_acceleration.x) || !std::isfinite(linear_acceleration.y) ||
        !std::isfinite(angular_acceleration) || !std::isfinite(state.stamp))
        return fail("invalid_input");

    auto command = [&](double v, double w, const char *status, bool pure_rotation = false)
    {
        v = std::clamp(v, linear_velocity - control.max_acceleration * control.control_period,
                         linear_velocity + control.max_acceleration * control.control_period);
        if (pure_rotation && phase_ != MotionPhase::StartRotating)
        {
            // 参考 directionAlign：限制角速度幅值上升，收零不再经过对称减速限幅。
            double limit = std::abs(angular_velocity) + control.max_rotation_acceleration * control.control_period;
            w = std::clamp(w, -limit, limit);
        }
        else
        {
            double acceleration = pure_rotation ? control.max_rotation_acceleration : control.max_angular_acceleration;
            w = std::clamp(w, angular_velocity - acceleration * control.control_period,
                             angular_velocity + acceleration * control.control_period);
        }
        result.velocity = pure_rotation ? 0 : std::clamp(v, 0.0, control.max_velocity);
        result.angular_velocity = std::clamp(w, -control.max_angular_velocity, control.max_angular_velocity);
        result.success = true;
        LOGD(PLANNER_TAG) << "local plan " << status << " v=" << result.velocity << " w=" << result.angular_velocity
                        << " seed=" << initial_source << " reset=" << reset_reason
                        << " esdf_ms=" << esdf_ms << " elapsed_ms=" << elapsedMs(begin);
        return result;
    };
    auto stopped = [&]()
    {
        if (std::abs(linear_velocity) > control.stopped_velocity ||
            std::abs(angular_velocity) > control.stopped_angular_velocity)
        {
            stopped_since_ = -1;
            return false;
        }
        if (stopped_since_ < 0 || state.stamp < stopped_since_)
            stopped_since_ = state.stamp;
        return state.stamp - stopped_since_ >= control.stopped_duration;
    };

    double goal_distance = (pose - goal_).head<2>().norm();
    double progress = updateProgress(pose.head<2>());
    if (phase_ == MotionPhase::NewPath)
    {
        // 起步对齐使用本帧实测位姿，不使用全局搜索时的起点朝向。
        alignment_yaw_ = pathYaw(std::min(path_lengths_.back(), progress + param_.waypoint_spacing / 2),
                                 planar_pose.yaw);
        phase_ = std::abs(wrap(alignment_yaw_ - planar_pose.yaw)) > param_.rotate_threshold ?
            MotionPhase::StartRotating : MotionPhase::Navigating;
    }
    if ((phase_ == MotionPhase::Navigating || phase_ == MotionPhase::StartRotating) &&
        goal_distance <= param_.position_tolerance)
    {
        phase_ = MotionPhase::GoalRotating;
        alignment_yaw_ = goal_.z();
        previous_trajectory_ = {};
        rotation_ready_ = false;
        stopped_since_ = -1;
    }
    if (phase_ == MotionPhase::GoalRotating && goal_distance > param_.position_tolerance)
    {
        phase_ = MotionPhase::Navigating;
        previous_trajectory_ = {};
        force_search_ = false;
        rotation_ready_ = false;
        stopped_since_ = -1;
    }
    auto updateEsdf = [&]()
    {
        auto esdf_begin = std::chrono::steady_clock::now();
        if (cached_map_ != input.map)
        {
            esdf_ = std::make_shared<map::EsdfMap2D>(*input.map);
            cached_map_ = input.map;
        }
        esdf_ms = elapsedMs(esdf_begin);
    };
    if (phase_ == MotionPhase::StartRotating || phase_ == MotionPhase::GoalRotating)
    {
        bool initial = phase_ == MotionPhase::StartRotating;
        const char *status = initial ? "initial_rotating" : "goal_rotating";
        result.local_goal << pose.head<2>(), unwrap(alignment_yaw_, pose.z());
        // 入场先减速停稳；停稳后的旋转命令严格不含线速度。
        if (!rotation_ready_)
        {
            if (!stopped())
                return command(0, 0, initial ? "initial_braking" : "goal_braking");
            rotation_ready_ = true;
            stopped_since_ = -1;
        }
        double error = wrap(alignment_yaw_ - pose.z());
        double tolerance = initial ? param_.start_yaw_tolerance : param_.yaw_tolerance;
        if (std::abs(error) <= tolerance)
        {
            if (!stopped())
                return command(0, 0, status, true);
            phase_ = initial ? MotionPhase::Navigating : MotionPhase::Finished;
            previous_trajectory_ = {};
            force_search_ = false;
            rotation_ready_ = false;
            stopped_since_ = -1;
        }
        else
        {
            stopped_since_ = -1;
            updateEsdf();
            // 当前机器人净距按外接圆计算，原地旋转不改变圆形包络。
            double clearance = 0;
            robot_->clearance(planar_pose, *esdf_, clearance);
            if (!std::isfinite(clearance) || clearance < 0)
                return fail("rotation_collision");
            return command(0, control.heading_gain * error, status, true);
        }
    }
    if (phase_ == MotionPhase::Finished)
    {
        result.goal_reached = true;
        result.local_goal = goal_;
        return command(0, 0, "goal_reached", true);
    }

    updateEsdf();
    double target_distance = std::min(path_lengths_.back(), progress + param_.lookahead_distance);
    Eigen::Vector3d target = pose;
    bool target_found = false;
    while (target_distance >= progress)
    {
        target = pathPoint(target_distance);
        target.z() = unwrap(pathYaw(target_distance, pose.z()), pose.z());
        if (!input.map->occupied({target.x(), target.y(), 0}))
        {
            target_found = true;
            break;
        }
        target_distance -= input.map->resolution();
    }
    if (!target_found) return fail("no_free_lookahead");
    auto &plan = *optimizer_input_;
    plan.touches_goal = target_distance >= path_lengths_.back() - 1e-6;
    // 每次起点完全使用外部实测状态；旧轨迹只提供中间点和时间初值。
    plan.start.position = pose;
    plan.start.velocity << linear_velocity * std::cos(pose.z()), linear_velocity * std::sin(pose.z()), angular_velocity;
    // 输入是车体系中的运动加速度，旋转到世界系；向心分量已在输入中，不重复叠加。
    plan.start.acceleration << linear_acceleration.x * std::cos(pose.z()) - linear_acceleration.y * std::sin(pose.z()),
                               linear_acceleration.x * std::sin(pose.z()) + linear_acceleration.y * std::cos(pose.z()), angular_acceleration;
    plan.map = input.map;
    plan.esdf = esdf_;
    plan.start_time = state.stamp;
    plan.goal.position = target;

    // 几何路径仅在构造初值时简化；不据此否决热启动或验收优化结果。
    auto geometricSeed = [&](const std::vector<Eigen::Vector3d> &route)
    {
        std::vector<Eigen::Vector3d> simplified{route.front()};
        for (std::size_t i = 0; i + 1 < route.size();)
        {
            std::size_t next = route.size() - 1;
            while (next > i + 1 && !lineFree(*input.map, route[i], route[next])) --next;
            if ((route[next] - simplified.back()).head<2>().norm() > 1e-6) simplified.push_back(route[next]);
            i = next;
        }
        std::vector<Eigen::Vector3d> knots{pose};
        for (std::size_t i = 1; i < simplified.size(); ++i)
        {
            int count = std::max(1, static_cast<int>(std::ceil(
                (simplified[i] - simplified[i - 1]).head<2>().norm() / param_.waypoint_spacing)));
            for (int j = 1; j <= count; ++j)
                knots.push_back(simplified[i - 1] + (simplified[i] - simplified[i - 1]) * (static_cast<double>(j) / count));
        }
        if (knots.size() < 2) return false;
        double yaw = pose.z();
        for (std::size_t i = 1; i < knots.size(); ++i)
        {
            std::size_t left = i - 1, right = i;
            double length = (knots[i] - knots[left]).head<2>().norm();
            while (left > 0 && length < param_.waypoint_spacing)
            {
                length += (knots[left] - knots[left - 1]).head<2>().norm();
                --left;
            }
            if (i + 1 < knots.size()) right = i + 1;
            Eigen::Vector2d delta = (knots[right] - knots[left]).head<2>();
            if (delta.norm() > 1e-8) yaw = unwrap(std::atan2(delta.y(), delta.x()), yaw);
            knots[i].z() = yaw;
        }
        // 全局切片和 A* 均使用路径切线；最终目标 yaw 只在停稳后单独处理。
        plan.goal.position = knots.back();
        plan.inner_points.resize(3, knots.size() - 2);
        plan.durations.resize(knots.size() - 1);
        for (std::size_t i = 1; i < knots.size(); ++i)
        {
            if (i + 1 < knots.size()) plan.inner_points.col(i - 1) = knots[i];
            plan.durations[i - 1] = std::max({0.3, (knots[i] - knots[i - 1]).head<2>().norm() / param_.cruise_speed,
                std::abs(knots[i].z() - knots[i - 1].z()) / (0.5 * control.max_angular_velocity)});
        }
        return true;
    };
    bool searched = false;
    auto searchSeed = [&]()
    {
        searched = true;
        *search_input_ = {{pose.x(), pose.y(), 0}, {target.x(), target.y(), 0}, input.map, 0};
        auto route = searcher_->search();
        initial_source = "astar";
        return route.success && geometricSeed(route.path);
    };

    double passed = state.stamp - previous_start_time_;
    double remaining = previous_trajectory_.getPieceNum() > 0 ? previous_trajectory_.getTotalDuration() - passed : 0;
    if (force_search_)
    {
        if (!searchSeed()) return fail("local_search_failed");
    }
    else if (previous_trajectory_.getPieceNum() > 0 && passed >= 0 && remaining > 0)
    {
        // df_a：裁去已执行部分，保留剩余多项式，前瞻点推进后接全局路径尾段。
        double extension = std::max(0.0, target_distance - previous_target_distance_) / param_.cruise_speed;
        int pieces = std::max(2, static_cast<int>(std::ceil(param_.lookahead_distance / param_.waypoint_spacing)));
        double duration = std::max(0.3 * pieces, remaining + extension);
        plan.durations = Eigen::VectorXd::Constant(pieces, duration / pieces);
        plan.inner_points.resize(3, pieces - 1);
        auto referencePoint = [&](double t) -> Eigen::Vector3d
        {
            if (t <= remaining)
                return previous_trajectory_.getPos(passed + t);
            double distance = std::min(target_distance, previous_target_distance_ + (t - remaining) * param_.cruise_speed);
            Eigen::Vector3d point = pathPoint(distance);
            point.z() = pathYaw(distance, pose.z());
            return point;
        };
        double yaw = pose.z();
        for (int i = 1; i < pieces; ++i)
        {
            double t = duration * i / pieces;
            Eigen::Vector3d point = referencePoint(t);
            point.z() = yaw = unwrap(point.z(), yaw);
            plan.inner_points.col(i - 1) = point;
        }
        double tail_time = std::max(0.0, remaining + extension - param_.waypoint_spacing / param_.cruise_speed);
        Eigen::Vector2d tail = (target - referencePoint(tail_time)).head<2>();
        plan.goal.position.z() = unwrap(tail.norm() > 1e-8 ? std::atan2(tail.y(), tail.x()) : target.z(), yaw);
        initial_source = "warm_start";
    }
    else
    {
        std::vector<Eigen::Vector3d> global_slice{pose, pathPoint(progress)};
        for (std::size_t i = 0; i < global_path_.size(); ++i)
            if (path_lengths_[i] > progress && path_lengths_[i] < target_distance)
                global_slice.push_back(global_path_[i]);
        global_slice.push_back(target);
        initial_source = "global_slice";
        if (!geometricSeed(global_slice)) return fail("empty_global_slice");
    }

    // 一次常规优化（内部最多三次）；失败后只搜索一次，再调用一次优化器。
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        plan.goal.velocity.setZero();
        plan.goal.acceleration.setZero();
        if (!plan.touches_goal)
            plan.goal.velocity << param_.cruise_speed * std::cos(plan.goal.position.z()),
                                  param_.cruise_speed * std::sin(plan.goal.position.z()), 0;
        poly_traj::MinJerkOpt<3> seed;
        seed.reset(plan.start.matrix(), plan.goal.matrix(), plan.durations.size());
        seed.generate(plan.inner_points, plan.durations);
        result.initial_trajectory = seed.getTraj();
        auto optimized = optimizer_->optimize();
        result.success = optimized.success;
        if (result.success)
        {
            result.trajectory = std::move(optimized.trajectory);
            break;
        }
        reset_reason = "optimization_failed";
        discardWarmStart();
        if (searched) break;
        if (!searchSeed()) return fail("local_search_failed");
    }
    if (!result.success) return fail("local_optimization_failed");
    result.local_goal = plan.goal.position;
    previous_trajectory_ = result.trajectory;
    previous_start_time_ = state.stamp;
    previous_target_distance_ = target_distance;
    force_search_ = false;
    double sample = 0, distance = 0;
    auto durations = result.trajectory.getDurations();
    int intervals = optimizer_->param().integration_intervals;
    int last = result.trajectory.getPieceNum() * intervals;
    int check_end = plan.touches_goal ? last : last - (last - 1) / 3;
    int end_piece = std::min(check_end / intervals, result.trajectory.getPieceNum() - 1);
    double tracking_end = durations.head(end_piece).sum() + durations[end_piece] * (check_end - end_piece * intervals) / intervals;
    Eigen::Vector3d previous = result.trajectory.getPos(0);
    while (sample < tracking_end && distance < control.tracking_distance)
    {
        sample = std::min(sample + 0.05, tracking_end);
        Eigen::Vector3d point = result.trajectory.getPos(sample);
        distance += (point - previous).head<2>().norm();
        previous = point;
    }
    Eigen::Vector3d desired = result.trajectory.getPos(sample), velocity = result.trajectory.getVel(sample);
    double c = std::cos(pose.z()), s = std::sin(pose.z());
    Eigen::Vector2d error = (desired - pose).head<2>();
    double forward = std::cos(desired.z()) * velocity.x() + std::sin(desired.z()) * velocity.y();
    double bearing = wrap(std::atan2(error.y(), error.x()) - pose.z());
    double v = std::min(param_.cruise_speed, std::max(0.0, forward) +
        control.position_gain * std::max(0.0, c * error.x() + s * error.y()));
    v *= std::max(0.0, std::cos(bearing));
    if (plan.touches_goal)
        v = std::min(v, std::sqrt(2 * control.max_acceleration * std::max(0.0, goal_distance - param_.position_tolerance / 2)));
    double w = control.lateral_gain * v * (-s * error.x() + c * error.y()) / std::max(0.01, error.squaredNorm());
    return command(v, w, "navigating");
}
} // namespace navapex::planner
