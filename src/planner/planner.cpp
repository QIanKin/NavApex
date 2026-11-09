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

double unwrap(double angle, double reference)
{
    return reference + wrap(angle - reference);
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
    force_search_   = true;
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
    search_input_->ignore_goal_obstacles = false;
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
    std::size_t index = std::max<std::size_t>(1, next - path_lengths_.begin());
    // point = p(i-1) + u * [p(i) - p(i-1)]
    // u = ( distance - path_length[i-1] )/ length
    double length = path_lengths_[index] - path_lengths_[index - 1];
    double u = length > 1e-9 ? (distance - path_lengths_[index - 1]) / length : 0;
    return global_path_[index - 1] + u * (global_path_[index] - global_path_[index - 1]);
}

//计算全局路径上一个点的大致朝向
//采用 point-windows/2 -> point+windows/2
double Planner::pathYaw(double distance, double fallback_yaw) const
{
    double window = param_.waypoint_spacing;
    double lower = std::max(0.0, std::min(distance - window / 2, path_lengths_.back() - window));
    double upper = std::min(lower + window, path_lengths_.back());
    Eigen::Vector2d delta = (pathPoint(upper) - pathPoint(lower)).head<2>();
    return delta.norm() > 1e-9 ? std::atan2(delta.y(), delta.x()) : fallback_yaw;
}

LocalPlanResult Planner::planLocal(const LocalPlanInput &input)
{
    // 0. 坐标系转换和初值处理
    const RobotState &state = input.state;
    Pose2D planar_pose = toPose2D(state.pose);
    Eigen::Vector3d pose(planar_pose.x, planar_pose.y, planar_pose.yaw);
    double linear_velocity = state.twist.linear.x;
    double angular_velocity = state.twist.angular.z;
    double acc_body_x = state.acceleration.linear.x;
    double acc_body_y = state.acceleration.linear.y;
    double vel_world_x = linear_velocity * std::cos(pose.z());
    double vel_world_y = linear_velocity * std::sin(pose.z());
    double acc_world_x = acc_body_x * std::cos(pose.z()) - acc_body_y * std::sin(pose.z());
    double acc_world_y = acc_body_x * std::sin(pose.z()) + acc_body_y * std::cos(pose.z());
    double acc_world_z = state.acceleration.angular.z;

    Eigen::Vector3d velocity(vel_world_x, vel_world_y, angular_velocity);
    Eigen::Vector3d acceleration(acc_world_x, acc_world_y, acc_world_z);

    const auto &control = param_;

    LocalPlanResult result;
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
            // 1. 更新progress 选取aim point
            double progress = updateProgress(pose.head<2>());
            double aim_distance = std::min(path_lengths_.back(), progress + param_.lookahead_distance);
            Eigen::Vector3d aim_point = pathPoint(aim_distance);
            aim_point.z() = unwrap(pathYaw(aim_distance, pose.z()), pose.z());

            // 2.组装基础输入
            auto &opt_input = *optimizer_input_;
            opt_input.touches_goal = aim_distance >= path_lengths_.back() - 1e-6;
            opt_input.start.position = pose;
            opt_input.start.velocity = velocity;
            opt_input.start.acceleration = acceleration;
            opt_input.map = input.map;
            opt_input.esdf = std::make_shared<map::EsdfMap2D>(*input.map);
            opt_input.start_time = state.stamp;
            opt_input.goal.position = aim_point;
            opt_input.goal.velocity.setZero();
            opt_input.goal.acceleration.setZero();
            if (!opt_input.touches_goal){
                opt_input.goal.velocity << param_.cruise_speed * std::cos(aim_point.z()),
                                           param_.cruise_speed * std::sin(aim_point.z()), 0;
            }

            // 3. 优先热启动
            double passed = state.stamp - previous_start_time_;
            double remaining = previous_trajectory_.getPieceNum() > 0 ? previous_trajectory_.getTotalDuration() - passed : 0;
            if (remaining > 0)
            {
                //热启动
                double extension_time = std::max(0.0, aim_distance - previous_aim_distance_) / param_.cruise_speed;
                double distance_to_aim = (aim_point - pose).head<2>().norm();
                int pieces = std::max(2, static_cast<int>(std::ceil(distance_to_aim / param_.waypoint_spacing)));
                double duration = remaining + extension_time;
                opt_input.durations = Eigen::VectorXd::Constant(pieces, duration / pieces);
                opt_input.inner_points.resize(3, pieces - 1);

                //组装航点
                double yaw = pose.z();
                for (int i = 1; i < pieces; ++i)
                {
                    double t = duration * i / pieces;
                    Eigen::Vector3d point;
                    if (t <= remaining)
                        point = previous_trajectory_.getPos(passed + t);
                    else
                    {
                        double distance = std::min(aim_distance, previous_aim_distance_ + (t - remaining) * param_.cruise_speed);
                        point = pathPoint(distance);
                        point.z() = pathYaw(distance, pose.z());
                    }
                    yaw = unwrap(point.z(), yaw); //保证yaw角连续不跳变
                    point.z() = yaw;
                    opt_input.inner_points.col(i - 1) = point;
                }

                //调用优化算法
                opt_input.goal.velocity.setZero();
                opt_input.goal.acceleration.setZero();
                if (!opt_input.touches_goal){
                    opt_input.goal.velocity << param_.cruise_speed * std::cos(aim_point.z()),
                                               param_.cruise_speed * std::sin(aim_point.z()), 0;
                }
                auto optimized = optimizer_->optimize();
                result.success = optimized.success;
                if (result.success){
                    result.trajectory = std::move(optimized.trajectory);
                }else{
                    force_search_ = true;
                }
            }else{
                force_search_ = true;
            }

            // 4. 无可用旧轨迹或热启动失败时，调用astar提供初值
            if (force_search_)
            {
                force_search_ = false;
                previous_trajectory_ = {};
                // 调用astar搜索
                *search_input_ = {{pose.x(), pose.y(), 0}, {aim_point.x(), aim_point.y(), 0}, input.map, robot_->radius()};
                if (!opt_input.touches_goal)
                    search_input_->ignore_goal_obstacles = true;
                auto search_result = searcher_->search();
                if (!search_result.success)
                {
                    LOGW(PLANNER_TAG) << "local search failed";
                    return result;
                }
                const auto &route = search_result.path;
                if (route.size() < 2)
                {
                    LOGW(PLANNER_TAG) << "local search returned an invalid path";
                    return result;
                }
                std::vector<double> lengths(route.size(), 0.0);
                // 计算累计弧长
                for (std::size_t i = 1; i < route.size(); ++i)
                {
                    lengths[i] = lengths[i - 1] + (route[i] - route[i - 1]).head<2>().norm();
                }
                if (lengths.back() <= 1e-6)
                {
                    LOGW(PLANNER_TAG) << "local search returned a zero-length path";
                    return result;
                }
                // 初值巡航时间 T = length / pieces / cruise_speed
                int pieces = std::max(2, static_cast<int>(std::ceil(lengths.back() / param_.waypoint_spacing)));
                double spacing = lengths.back() / pieces;
                opt_input.inner_points.resize(3, pieces - 1);
                opt_input.durations = Eigen::VectorXd::Constant(pieces, spacing / param_.cruise_speed);

                // lambda 计算累计距离对应的point
                auto routePoint = [&](double distance) -> Eigen::Vector3d
                {
                    if (distance >= lengths.back())
                        return route.back();
                    std::size_t i = std::upper_bound(lengths.begin(), lengths.end(), distance) - lengths.begin();
                    while (i < lengths.size() && lengths[i] <= lengths[i - 1]) ++i;
                    if (i >= lengths.size()) return route.back();
                    double segment_length = lengths[i] - lengths[i - 1];
                    double u = (distance - lengths[i - 1]) / segment_length;
                    return route[i - 1] + u * (route[i] - route[i - 1]);
                };

                // 组装航点
                double yaw = pose.z();
                for (int i = 1; i <= pieces; ++i)
                {
                    double distance = spacing * i;
                    Eigen::Vector3d point = routePoint(distance);
                    double lower = std::max(0.0, distance - spacing / 2);
                    double upper = std::min(lengths.back(), distance + spacing / 2);
                    Eigen::Vector2d tangent = (routePoint(upper) - routePoint(lower)).head<2>();
                    if (tangent.norm() > 1e-8)
                        yaw = unwrap(std::atan2(tangent.y(), tangent.x()), yaw);
                    point.z() = yaw;
                    if (i < pieces){
                        opt_input.inner_points.col(i - 1) = point;
                    }else{
                        opt_input.goal.position = point;
                    }
                }
                if (!opt_input.touches_goal)
                    opt_input.goal.velocity << param_.cruise_speed * std::cos(yaw),
                                               param_.cruise_speed * std::sin(yaw), 0;
                auto optimized = optimizer_->optimize();
                result.success = optimized.success;
                if(result.success){
                    result.trajectory = std::move(optimized.trajectory);
                }
            }

            if (!result.success)
            {
                LOGW(PLANNER_TAG) << "local plan failed: local_optimization_failed";
                return result;
            }
            previous_trajectory_ = result.trajectory;
            previous_start_time_ = state.stamp;
            previous_aim_distance_ = aim_distance;
            force_search_ = false;
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
        LOGI(PLANNER_TAG) << "goal reachded!";
        return result;
    }

    return result;
}
} // namespace navapex::planner
