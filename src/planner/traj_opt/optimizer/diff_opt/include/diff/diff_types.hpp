#pragma once

#include <Eigen/Core>
#include <optimizer_io.hpp>
#include <utils/poly_traj_utils.hpp>
#include <vector>

namespace navapex::traj_opt
{
struct DiffCost
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

struct DiffPlanningState
{
    std::shared_ptr<const OptimizerInput> input;

    int piece_count = 0;
    Eigen::Matrix3Xd sample_points{3, 0};
    double swarm_weight = 0;

    bool stop = false;
    DiffCost evaluation_cost;
    std::vector<double> min_swarm_dist_sq;
};

} // namespace navapex::traj_opt
