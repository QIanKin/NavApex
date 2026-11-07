#pragma once

#include <Eigen/Core>
#include <perception/maps/base_map.hpp>
#include <memory>
#include <robot_model/robot_model.hpp>
#include <utils/poly_traj_utils.hpp>
#include <vector>

namespace navapex::traj_opt
{

struct TrajectoryBoundary
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
    poly_traj::Trajectory<3> trajectory;
};

struct OptimizerInput
{
    TrajectoryBoundary start;
    TrajectoryBoundary goal;
    Eigen::Matrix3Xd inner_points{3, 0};
    Eigen::VectorXd durations;

    std::shared_ptr<const perception::OccupancyMap> map;
    std::shared_ptr<const map::EsdfMap2D> esdf;
    std::shared_ptr<const robot_model::RobotModel> robot_;
    std::vector<NeighborTrajectory> neighbors;
    double start_time = 0;
    int robot_id = 0;
    bool touches_goal = false;
};

struct OptimizerResult
{
    bool success = false;
    poly_traj::Trajectory<3> trajectory;
};

} // namespace navapex::traj_opt
