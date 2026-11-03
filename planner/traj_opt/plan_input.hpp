#pragma once

#include <Eigen/Core>
#include <map/occupancy_map.hpp>
#include <memory>
#include <utils/poly_traj_utils.hpp>
#include <vector>

namespace traj_opt
{
struct State
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

    Eigen::Matrix3d matrix() const
    {
        Eigen::Matrix3d m;
        m << position, velocity, acceleration;
        return m;
    }
};

struct NeighborTrajectory
{
    int robot_id = -1;
    double start_time = 0;
    double clearance = 0.15;
    poly_traj::Trajectory trajectory;
};

struct PlanInput
{
    State start;
    State goal;
    Eigen::Matrix3Xd inner_points{3, 0};
    Eigen::VectorXd durations;

    std::shared_ptr<const map::OccupancyMap> map;
    std::vector<NeighborTrajectory> neighbors;
    double start_time = 0;
    int robot_id = 0;
    bool touches_goal = true;
};

} // namespace traj_opt
