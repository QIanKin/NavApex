#pragma once

#include <Eigen/Core>
#include <plan_input.hpp>
#include <utils/poly_traj_utils.hpp>
#include <vector>

namespace diff
{
struct Cost
{
    double smoothness = 0;
    double time = 0;
    double obstacle = 0;
    double obstacle_soft = 0;
    double velocity = 0;
    double acceleration = 0;
    double angular_velocity = 0;
    double angular_acceleration = 0;
    double kinematics = 0;
    double spacing = 0;
    double swarm = 0;
    double total() const
    {
        return smoothness + time + obstacle + obstacle_soft +
            velocity + acceleration + angular_velocity + angular_acceleration +
            kinematics + spacing + swarm;
    }
};

enum class StopReason
{
    None,
    Error
};

struct PlanningState
{
    std::shared_ptr<const traj_opt::PlanInput> input;

    int piece_count = 0;
    Eigen::Matrix3Xd sample_points{3, 0};
    double swarm_weight = 0;
    int evaluations = 0;

    StopReason stop_reason = StopReason::None;
    Cost evaluation_cost;
    std::vector<double> min_swarm_dist_sq;
};

} // namespace diff
