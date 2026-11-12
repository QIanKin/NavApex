#include "planner.hpp"
#include "planner/path_plan/searcher_io.hpp"
#include "robot_model/robot_state.hpp"
#include <robot_model/robot_model.hpp>

#include <perception/esdf/esdf_map_2d.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
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

DiffPlanner::DiffPlanner(std::shared_ptr<const robot_model::RobotModel> robot, PlannerParams param)
                        : robot_(std::move(robot))
{
    param_ = std::move(param);
    search_input_ = std::make_shared<path_plan::SearchInput>();
    searcher_ = std::make_unique<path_plan::SearcherInterface>("astar2d", search_input_);
    optimizer_input_ = std::make_shared<traj_opt::OptimizerInput>();
    optimizer_ = std::make_unique<traj_opt::OptimizerInterface>("diff", optimizer_input_);
    optimizer_input_->robot_ = robot_;
}

void DiffPlanner::cancelWarmStart()
{
    previous_trajectory_ = {};
}

const PlannerParams &DiffPlanner::param() const
{ 
    return param_; 
}

void DiffPlanner::reset()
{
    phase_ = MotionPhase::NewPath;
    global_path_.clear();
    path_lengths_.clear();
    previous_trajectory_ = {};
    force_search_   = true;
    alignment_yaw_  = 0;
    previous_aim_distance_ = 0;
}

GlobalPlanResult DiffPlanner::planGlobal(const GlobalPlanInput &input)
{
    //初始化并搜索全局路径
    reset();
    GlobalPlanResult result;
    search_input_->start = input.start.position;
    search_input_->goal  = input.goal.position;
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
    global_goal_ = input.goal.toPlanar().toEigen();
    global_path_.reserve(search_result.path.size());
    for (const auto &point : search_result.path)
    {
        global_path_.emplace_back(point.toPlanar().toEigen());
    }
    path_lengths_.resize(global_path_.size());
    path_lengths_[0] = 0;
    for (std::size_t i = 1; i < global_path_.size(); ++i)
    {
        path_lengths_[i] = path_lengths_[i - 1] + (global_path_[i] - global_path_[i - 1]).norm();
    }
    result.path = std::move(search_result.path);
    result.success = true;
    LOGI(PLANNER_TAG) << "global plan success";
    return result;
}

double DiffPlanner::findAimDistance(const Eigen::Vector2d &position) const
{
    const double step = param_.lookahead_distance / 20;
    for (double distance = previous_aim_distance_; distance < path_lengths_.back(); distance += step)
    {
        if ((pathPoint(distance) - position).norm() >= param_.lookahead_distance)
            return distance;
    }
    return path_lengths_.back();
}

Eigen::Vector2d DiffPlanner::pathPoint(double distance) const
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

double DiffPlanner::pathYaw(double distance, double fallback_yaw) const
{
    // 计算全局路径上一个点的大致朝向
    // point-windows/2 -> point+windows/2 
    double window = param_.waypoint_spacing;
    double lower = std::max(0.0, std::min(distance - window / 2, path_lengths_.back() - window));
    double upper = std::min(lower + window, path_lengths_.back());
    Eigen::Vector2d delta = pathPoint(upper) - pathPoint(lower);
    return delta.norm() > 1e-9 ? std::atan2(delta.y(), delta.x()) : fallback_yaw;
}

LocalPlanResult DiffPlanner::planLocal(const LocalPlanInput &input)
{
    // 0. 初值处理
    const RobotState &state = input.state;
    Eigen::Vector3d pose = state.pose.toPlanar().toEigen();
    double start_linear_velocity = state.twist.toPlanar().linear.x;
    double start_angular_velocity = state.twist.toPlanar().angular.z;

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
            Eigen::Vector2d segment = global_path_[i] - global_path_[i - 1];
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
        if(std::abs(start_linear_velocity) > 0.1 || std::abs(start_angular_velocity) >0.1)
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
            double w = param_.heading_gain * error;
            double limit = std::abs(start_angular_velocity) + param_.max_rotation_acceleration * param_.control_period;
            w = std::clamp(w, -limit, limit);
            result.velocity = 0;
            result.angular_velocity = std::clamp(w, -param_.max_rotation_velocity, param_.max_rotation_velocity);
            result.success = true;
            return result;
        }
    }

    if(phase_ == MotionPhase::Navigating)
    {
        double goal_distance = (pose.head<2>() - global_goal_.head<2>()).norm();
        if (goal_distance <= param_.position_tolerance)
        {
            alignment_yaw_ = global_goal_.z();
            previous_trajectory_ = {};
            phase_ = MotionPhase::GoalRotating;
            LOGI(PLANNER_TAG) << "goal position reached, align direction";
        }
        else
        {
            // 1. 从上次目标继续沿全局路径选取 aim point。
            // TODO 沿着全局轨迹维护progress 往前找lookahead 两次distance距离加剩余轨迹段数来算pieces
            double aim_distance = findAimDistance(pose.head<2>());
            Eigen::Vector3d aim_point;
            aim_point.head<2>() = pathPoint(aim_distance);
            aim_point.z() = unwrap(pathYaw(aim_distance, pose.z()), pose.z());

            // 2.组装基础输入
            auto &opt_input = *optimizer_input_;
            opt_input.touches_goal = aim_distance >= path_lengths_.back() - 1e-6;
            opt_input.start.q = pose;
            opt_input.start.dq = state.twist.toPlanar().toWorld(state.pose).toEigen();
            opt_input.start.ddq = state.acceleration.toPlanar().toWorld(state.pose).toEigen();
            opt_input.map = input.map;
            opt_input.esdf = std::make_shared<map::EsdfMap2D>(*input.map);
            opt_input.start_time = state.stamp;
            opt_input.goal.q = aim_point;
            opt_input.goal.dq.setZero();
            opt_input.goal.ddq.setZero();
            if (!opt_input.touches_goal){
                opt_input.goal.dq << param_.cruise_speed * std::cos(aim_point.z()),
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
                        point.head<2>() = pathPoint(distance);
                        point.z() = pathYaw(distance, pose.z());
                    }
                    yaw = unwrap(point.z(), yaw); //保证yaw角连续不跳变
                    point.z() = yaw;
                    opt_input.inner_points.col(i - 1) = point;
                }

                //调用优化算法
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
                search_input_->start = state.pose.position;
                search_input_->goal = Position3D::fromEigen(aim_point);
                search_input_->map = input.map;
                search_input_->radius = robot_->radius();
                if (!opt_input.touches_goal)
                    search_input_->ignore_goal_obstacles = true;
                auto search_result = searcher_->search();
                if (!search_result.success)
                {
                    LOGW(PLANNER_TAG) << "local search failed";
                    return result;
                }
                auto &route = search_result.path;
                if (route.size() < 2)
                {
                    LOGW(PLANNER_TAG) << "local search returned an invalid path";
                    return result;
                }
                std::vector<double> lengths(route.size(), 0.0);
                // 计算累计弧长
                for (std::size_t i = 1; i < route.size(); ++i)
                {
                    lengths[i] = lengths[i - 1] + (route[i].toEigen() - route[i - 1].toEigen()).head<2>().norm();
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
                        return route.back().toEigen();
                    auto next = std::upper_bound(lengths.begin(), lengths.end(), distance);
                    std::size_t index = std::max<std::size_t>(1, next - lengths.begin());
                    double length = lengths[index] - lengths[index - 1];
                    double u = length > 1e-9 ? (distance - lengths[index - 1]) / length : 0;
                    return route[index - 1].toEigen() + u * (route[index].toEigen() - route[index - 1].toEigen());
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
                        opt_input.goal.q = point;  //目标yaw角可能由于astar改变 需要重新组装
                    }
                }
                if (!opt_input.touches_goal)
                {
                    opt_input.goal.dq << param_.cruise_speed * std::cos(yaw),
                                         param_.cruise_speed * std::sin(yaw), 0;
                }
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
        if(std::abs(start_linear_velocity) > 0.1 || std::abs(start_angular_velocity) >0.1)
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
            double w = param_.heading_gain * error;
            double limit = std::abs(start_angular_velocity) + param_.max_rotation_acceleration * param_.control_period;
            w = std::clamp(w, -limit, limit);
            result.velocity = 0;
            result.angular_velocity = std::clamp(w, -param_.max_rotation_velocity, param_.max_rotation_velocity);
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
