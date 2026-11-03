#pragma once

#include <Eigen/Core>
#include <plan_input.hpp>
#include <utils/poly_traj_utils.hpp>
#include <vector>

namespace omni3d
{
struct Cost
{
    double smoothness = 0;
    double time = 0;
    double obstacle = 0;
    double obstacle_soft = 0;
    double velocity = 0;
    double acceleration = 0;
    double jerk = 0;
    double spacing = 0;
    double swarm = 0;
    double total() const
    {
        return smoothness + time + obstacle + obstacle_soft +
            velocity + acceleration + jerk + spacing + swarm;
    }
};

struct ObstacleConstraint
{
    Eigen::Vector3d base;
    Eigen::Vector3d direction;
};

struct NormalPlane
{
    Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 2> basis = Eigen::Matrix<double, 3, 2>::Zero();
};

enum class StopReason
{
    None,
    Rebound,
    Error
};

struct PlanningState
{
    std::shared_ptr<const traj_opt::PlanInput> input;

    std::vector<NormalPlane> normal_planes;
    std::vector<std::vector<ObstacleConstraint>> obstacle_constraints;
    
    int piece_count = 0;
    Eigen::Matrix3Xd sample_points{3, 0};
    double swarm_weight = 0;
    int rebounds = 0;
    int evaluations = 0;
    
    StopReason stop_reason = StopReason::None;
    Cost evaluation_cost;
    std::vector<double> min_swarm_dist_sq;
};

} // namespace omni3d
