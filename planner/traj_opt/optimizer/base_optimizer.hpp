#pragma once

#include <utils/poly_traj_utils.hpp>

namespace traj_opt
{
class OptimizerParams;

struct Result
{
    bool success = false;
    poly_traj::Trajectory<3> trajectory;
    double total_cost = 0;
    double elapsed_ms = 0;
};

class Optimizer
{
public:
    virtual ~Optimizer() = default;

    virtual Result optimize() = 0;

    virtual const OptimizerParams &param() const = 0;

};

} // namespace traj_opt
