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
    Pose3D start;
    Pose3D goal;
};

struct GlobalPlanResult
{
    bool success = false;
    std::vector<Position3D> path;
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
    poly_traj::Trajectory<3> trajectory;
};

} // namespace navapex::planner
