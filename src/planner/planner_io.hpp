#pragma once

#include <perception/maps/base_map.hpp>
#include <robot_model/robot_state.hpp>
#include <memory>
#include <utils/poly_traj_utils.hpp>
#include <vector>

namespace navapex::planner
{

struct GlobalPlanInput
{
    std::shared_ptr<const perception::OccupancyMap> map;
    Pose2D start;
    Pose2D goal;
};

struct GlobalPlanResult
{
    bool success = false;
    std::vector<Eigen::Vector3d> path;
};

struct LocalPlanInput
{
    std::shared_ptr<const perception::OccupancyMap> map;
    RobotState state;
};

struct LocalPlanResult
{
    bool success = false;
    bool goal_reached = false;
    double velocity = 0;
    double angular_velocity = 0;
    Eigen::Vector3d local_goal = Eigen::Vector3d::Zero();
    poly_traj::Trajectory<3> trajectory;
    poly_traj::Trajectory<3> initial_trajectory;
};

} // namespace navapex::planner
