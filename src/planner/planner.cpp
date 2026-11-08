#include "planner.hpp"
#include "planner/path_plan/searcher_io.hpp"
#include "robot_model/robot_state.hpp"

#include <perception/esdf/esdf_map_2d.hpp>
#include <algorithm>
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

double wrap(double angle)
{
    if (angle > PI){
        angle -= 2 * PI;
    } else if (angle < -PI){
        angle += 2 * PI;
    }
    return angle;
}

double unwrap(double angle, double reference) { return reference + wrap(angle - reference); }

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
    alignment_yaw_  = 0;
    progress_ = 0;
}

GlobalPlanResult Planner::planGlobal(const GlobalPlanInput &input)
{
    //初始化并搜索全局路径
    reset();
    GlobalPlanResult result;
    search_input_->start = toPosition3D(input.start);
    search_input_->goal  = toPosition3D(input.goal);
    search_input_->map   = input.map; 
    search_input_->radius = robot_->radius();
    auto search_result = searcher_->search();
    if (!search_result.success)
    {
        LOGW(PLANNER_TAG) << "global search failed";
        return result;
    }
    //计算路径累加距离
    global_path_ = std::move(search_result.path);
    goal_ = Eigen::Vector3d(input.goal.x, input.goal.y, input.goal.yaw);
    path_lengths_.resize(global_path_.size());
    path_lengths_[0] = 0;
    for (std::size_t i = 1; i < global_path_.size(); ++i)
    {
        path_lengths_[i] = path_lengths_[i - 1] + (global_path_[i] - global_path_[i - 1]).head<2>().norm();
    }
    result.path = global_path_;
    result.success = true;
    LOGI(PLANNER_TAG) << "global plan success";
    return result;
}

double Planner::updateProgress(const Eigen::Vector2d &position)
{
    double best = std::numeric_limits<double>::infinity();
    double selected = progress_;
    for (std::size_t i = 1; i < global_path_.size(); ++i)
    {
        if (path_lengths_[i] < progress_ || path_lengths_[i - 1] > progress_ + 2 * param_.lookahead_distance)
            continue;
        Eigen::Vector2d a = global_path_[i - 1].head<2>();
        Eigen::Vector2d edge = global_path_[i].head<2>() - a;
        double length = edge.norm();
        if (length < 1e-9) 
            continue;
        // q = a + u * edge
        // u = (p - a) * edge / || edge ||^2
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
    LocalPlanResult result;
    const auto &state = input.state;
    auto planar_pose = toPose2D(state.pose);
    Eigen::Vector3d pose(planar_pose.x, planar_pose.y, planar_pose.yaw);
    double linear_velocity = state.twist.linear.x;
    double angular_velocity = state.twist.angular.z;
    const auto &linear_acceleration = state.acceleration.linear;
    double angular_acceleration = state.acceleration.angular.z;
    const auto &control = param_;

    if (phase_ == MotionPhase::NewPath)
    {
        if (global_path_.empty())
        {
            LOGW(PLANNER_TAG) << "local plan failed: no_global_path";
            return result;
        }
        //计算路径平均朝向
        Eigen::Vector2d direction = Eigen::Vector2d::Zero();
        double distance = 0;
        for (std::size_t i = 1; i < global_path_.size(); ++i)
        {
            Eigen::Vector2d segment = (global_path_[i] - global_path_[i - 1]).head<2>();
            double length = segment.norm();
            if (length < 1e-9) continue;
            direction += segment / length;
            distance += length;
            if (distance >= 0.6) break;
        }
        alignment_yaw_ = direction.norm() > 1e-9 ? std::atan2(direction.y(), direction.x()) : pose.z();
        if( std::abs(wrap(alignment_yaw_ - pose.z())) > param_.rotate_threshold){
            phase_ = MotionPhase::StartRotating;
            LOGI(PLANNER_TAG) << "global path behind in robot, start rotating";
        }else{
            phase_ = MotionPhase::Navigating;
            LOGI(PLANNER_TAG) << "start navigating";
        }
    }

    if(phase_ == MotionPhase::StartRotating)
    {
        if(std::abs(linear_velocity) > 0.1 || std::abs(angular_velocity) >0.1)
        {
            result.velocity = 0.0;
            result.angular_velocity = 0.0;
            result.success = true;
            LOGW(PLANNER_TAG) << "rotating need stop first";
            return result;
        }else{
            phase_ = MotionPhase::Rotating;
            LOGI(PLANNER_TAG) << "start rotating";
        }
    }

    if(phase_ == MotionPhase::Rotating)
    {
        double error = wrap(alignment_yaw_ - pose.z());
        if (std::abs(error) <= param_.start_yaw_tolerance){
            phase_ = MotionPhase::Navigating;
            LOGI(PLANNER_TAG) << "start navigating";
        }else{
            double w = control.heading_gain * error;
            double limit = std::abs(angular_velocity) + control.max_angular_acceleration * control.control_period;
            w = std::clamp(w, -limit, limit);
            result.velocity = 0;
            result.angular_velocity = std::clamp(w, -control.max_angular_velocity, control.max_angular_velocity);
            result.success = true;
            return result;
        }
    }

    if(phase_ == MotionPhase::Navigating)
    {
        double goal_distance = (pose - goal_).head<2>().norm();
        if (goal_distance <= param_.position_tolerance)
        {
            alignment_yaw_ = goal_.z();
            previous_trajectory_ = {};
            phase_ = MotionPhase::GoalRotating;
            LOGI(PLANNER_TAG) << "goal position reached, align direction";
        }
        else
        {
            // 1. 更新esdf
            if (cached_map_ != input.map)
            {
                esdf_ = std::make_shared<map::EsdfMap2D>(*input.map);
                cached_map_ = input.map;
            }
            double progress = updateProgress(pose.head<2>());
            double target_distance = std::min(path_lengths_.back(), progress + param_.lookahead_distance);
            Eigen::Vector3d target = pathPoint(target_distance);
            target.z() = unwrap(pathYaw(target_distance, pose.z()), pose.z());
            auto &opt_input = *optimizer_input_;
            opt_input.touches_goal = target_distance >= path_lengths_.back() - 1e-6;
            // 每次起点完全使用外部实测状态；旧轨迹只提供中间点和时间初值。
            opt_input.start.position = pose;
            opt_input.start.velocity << linear_velocity * std::cos(pose.z()), linear_velocity * std::sin(pose.z()), angular_velocity;
            // 输入是车体系中的运动加速度，旋转到世界系；向心分量已在输入中，不重复叠加。
            opt_input.start.acceleration << linear_acceleration.x * std::cos(pose.z()) - linear_acceleration.y * std::sin(pose.z()),
                                       linear_acceleration.x * std::sin(pose.z()) + linear_acceleration.y * std::cos(pose.z()), angular_acceleration;
            opt_input.map = input.map;
            opt_input.esdf = esdf_;
            opt_input.start_time = state.stamp;
            opt_input.goal.position = target;

            // 2. 优先热启动或全局切片；优化失败后只补一次 A* 搜索和优化。
            double passed = state.stamp - previous_start_time_;
            double remaining = previous_trajectory_.getPieceNum() > 0 ? previous_trajectory_.getTotalDuration() - passed : 0;
            bool searched = false;
            for (int attempt = 0; attempt < 2; ++attempt)
            {
                if (!force_search_ && previous_trajectory_.getPieceNum() > 0 && passed >= 0 && remaining > 0)
                {
                    // df_a：裁去已执行部分，保留剩余多项式，前瞻点推进后接全局路径尾段。
                    double extension = std::max(0.0, target_distance - previous_target_distance_) / param_.cruise_speed;
                    int pieces = std::max(2, static_cast<int>(std::ceil(param_.lookahead_distance / param_.waypoint_spacing)));
                    double duration = std::max(0.3 * pieces, remaining + extension);
                    opt_input.durations = Eigen::VectorXd::Constant(pieces, duration / pieces);
                    opt_input.inner_points.resize(3, pieces - 1);
                    double yaw = pose.z();
                    for (int i = 1; i < pieces; ++i)
                    {
                        double t = duration * i / pieces;
                        Eigen::Vector3d point;
                        if (t <= remaining)
                            point = previous_trajectory_.getPos(passed + t);
                        else
                        {
                            double distance = std::min(target_distance, previous_target_distance_ + (t - remaining) * param_.cruise_speed);
                            point = pathPoint(distance);
                            point.z() = pathYaw(distance, pose.z());
                        }
                        point.z() = yaw = unwrap(point.z(), yaw);
                        opt_input.inner_points.col(i - 1) = point;
                    }
                    double tail_time = std::max(0.0, remaining + extension - param_.waypoint_spacing / param_.cruise_speed);
                    Eigen::Vector3d tail_point;
                    if (tail_time <= remaining)
                        tail_point = previous_trajectory_.getPos(passed + tail_time);
                    else
                        tail_point = pathPoint(std::min(target_distance,
                            previous_target_distance_ + (tail_time - remaining) * param_.cruise_speed));
                    Eigen::Vector2d tail = (target - tail_point).head<2>();
                    opt_input.goal.position.z() = unwrap(tail.norm() > 1e-8 ? std::atan2(tail.y(), tail.x()) : target.z(), yaw);
                }
                else
                {
                    std::vector<Eigen::Vector3d> route;
                    if (force_search_)
                    {
                        searched = true;
                        *search_input_ = {{pose.x(), pose.y(), 0}, {target.x(), target.y(), 0}, input.map, 0};
                        auto search_result = searcher_->search();
                        if (!search_result.success)
                        {
                            discardWarmStart();
                            LOGW(PLANNER_TAG) << "local plan failed: local_search_failed";
                            return result;
                        }
                        route = std::move(search_result.path);
                    }
                    else
                    {
                        route = {pose, pathPoint(progress)};
                        for (std::size_t i = 0; i < global_path_.size(); ++i)
                            if (path_lengths_[i] > progress && path_lengths_[i] < target_distance)
                                route.push_back(global_path_[i]);
                        route.push_back(target);
                    }
                    // 简化几何路径并构造优化初值。
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
                    if (knots.size() < 2)
                    {
                        discardWarmStart();
                        LOGW(PLANNER_TAG) << "local plan failed: empty seed path";
                        return result;
                    }
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
                    opt_input.goal.position = knots.back();
                    opt_input.inner_points.resize(3, knots.size() - 2);
                    opt_input.durations.resize(knots.size() - 1);
                    for (std::size_t i = 1; i < knots.size(); ++i)
                    {
                        if (i + 1 < knots.size()) opt_input.inner_points.col(i - 1) = knots[i];
                        opt_input.durations[i - 1] = std::max({0.3, (knots[i] - knots[i - 1]).head<2>().norm() / param_.cruise_speed,
                            std::abs(knots[i].z() - knots[i - 1].z()) / (0.5 * control.max_angular_velocity)});
                    }
                }

                opt_input.goal.velocity.setZero();
                opt_input.goal.acceleration.setZero();
                if (!opt_input.touches_goal)
                    opt_input.goal.velocity << param_.cruise_speed * std::cos(opt_input.goal.position.z()),
                                          param_.cruise_speed * std::sin(opt_input.goal.position.z()), 0;
                poly_traj::MinJerkOpt<3> seed;
                seed.reset(opt_input.start.matrix(), opt_input.goal.matrix(), opt_input.durations.size());
                seed.generate(opt_input.inner_points, opt_input.durations);
                result.initial_trajectory = seed.getTraj();
                auto optimized = optimizer_->optimize();
                result.success = optimized.success;
                if (result.success)
                {
                    result.trajectory = std::move(optimized.trajectory);
                    break;
                }
                discardWarmStart();
                if (searched) break;
            }
            if (!result.success)
            {
                LOGW(PLANNER_TAG) << "local plan failed: local_optimization_failed";
                return result;
            }

            // 3. 保存热启动轨迹；导航控制交给外部轨迹跟踪器。
            result.local_goal = opt_input.goal.position;
            previous_trajectory_ = result.trajectory;
            previous_start_time_ = state.stamp;
            previous_target_distance_ = target_distance;
            force_search_ = false;
            result.success = true;
            return result;
        }
    }

    if(phase_ == MotionPhase::GoalRotating)
    {
        if(std::abs(linear_velocity) > 0.1 || std::abs(angular_velocity) >0.1)
        {
            result.velocity = 0.0;
            result.angular_velocity = 0.0;
            result.success = true;
            LOGW(PLANNER_TAG) << "rotating need stop first";
            return result;
        }else{
            phase_ = MotionPhase::DirectionAlign;
            LOGI(PLANNER_TAG) << "start rotating";
        }
    }

    if(phase_ == MotionPhase::DirectionAlign)
    {
        double error = wrap(alignment_yaw_ - pose.z());
        if (std::abs(error) <= param_.yaw_tolerance){
            phase_ = MotionPhase::Finished;
        }else{
            double w = control.heading_gain * error;
            double limit = std::abs(angular_velocity) + control.max_angular_acceleration * control.control_period;
            w = std::clamp(w, -limit, limit);
            result.velocity = 0;
            result.angular_velocity = std::clamp(w, -control.max_angular_velocity, control.max_angular_velocity);
            result.success = true;
            return result;
        }
    }

    if (phase_ == MotionPhase::Finished)
    {
        result.velocity = 0;
        result.angular_velocity = 0;
        result.success = true;
        result.goal_reached = true;
        result.local_goal = goal_;
        LOGI(PLANNER_TAG) << "goal reachded!";
        return result;
    }

    return result;
}
} // namespace navapex::planner
