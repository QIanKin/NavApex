#include "trajectory_tracker.hpp"

#include <params/diff_params.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>

namespace navapex::planner
{
TrajectoryTracker::TrajectoryTracker(const PlannerParams &param) : param_(param)
{
    auto path = std::filesystem::path(__FILE__).parent_path().parent_path() / "traj_opt" / "config" / "diff.json";
    integration_intervals_ = traj_opt::DiffParams(path.string()).integration_intervals;
}

VelocityCommand TrajectoryTracker::track(const LocalPlanResult &plan, const RobotState &state,
                                         const Eigen::Vector3d &goal) const
{
    VelocityCommand result;
    if (!plan.success || plan.goal_reached || plan.trajectory.getPieceNum() == 0)
        return result;

    const auto &control = param_;
    auto current = toPose2D(state.pose);
    Eigen::Vector3d pose(current.x, current.y, current.yaw);
    double linear_velocity = state.twist.linear.x, angular_velocity = state.twist.angular.z;
    double goal_distance = (goal - pose).head<2>().norm();
    bool touches_goal = (plan.local_goal - goal).head<2>().norm() <= 1e-6;

    double sample = 0, distance = 0;
    auto durations = plan.trajectory.getDurations();
    int intervals = integration_intervals_;
    int last = plan.trajectory.getPieceNum() * intervals;
    int check_end = touches_goal ? last : last - (last - 1) / 3;
    int end_piece = std::min(check_end / intervals, plan.trajectory.getPieceNum() - 1);
    double tracking_end = durations.head(end_piece).sum() + durations[end_piece] * (check_end - end_piece * intervals) / intervals;
    Eigen::Vector3d previous = plan.trajectory.getPos(0);
    while (sample < tracking_end && distance < control.tracking_distance)
    {
        sample = std::min(sample + 0.05, tracking_end);
        Eigen::Vector3d point = plan.trajectory.getPos(sample);
        distance += (point - previous).head<2>().norm();
        previous = point;
    }
    Eigen::Vector3d desired = plan.trajectory.getPos(sample), velocity = plan.trajectory.getVel(sample);
    double c = std::cos(pose.z()), s = std::sin(pose.z());
    Eigen::Vector2d error = (desired - pose).head<2>();
    double forward = std::cos(desired.z()) * velocity.x() + std::sin(desired.z()) * velocity.y();
    double bearing = std::atan2(error.y(), error.x()) - pose.z();
    bearing = std::atan2(std::sin(bearing), std::cos(bearing));
    double v = std::min(param_.cruise_speed, std::max(0.0, forward) +
        control.position_gain * std::max(0.0, c * error.x() + s * error.y()));
    v *= std::max(0.0, std::cos(bearing));
    if (touches_goal)
        v = std::min(v, std::sqrt(2 * control.max_acceleration * std::max(0.0, goal_distance - param_.position_tolerance / 2)));
    double w = control.lateral_gain * v * (-s * error.x() + c * error.y()) / std::max(0.01, error.squaredNorm());
    v = std::clamp(v, linear_velocity - control.max_acceleration * control.control_period,
                     linear_velocity + control.max_acceleration * control.control_period);
    w = std::clamp(w, angular_velocity - control.max_angular_acceleration * control.control_period,
                     angular_velocity + control.max_angular_acceleration * control.control_period);
    result.velocity = std::clamp(v, 0.0, control.max_velocity);
    result.angular_velocity = std::clamp(w, -control.max_angular_velocity, control.max_angular_velocity);
    return result;
}
} // namespace navapex::planner
