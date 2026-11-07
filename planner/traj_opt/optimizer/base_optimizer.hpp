#pragma once

#include <optimizer_io.hpp>

namespace navapex::traj_opt
{
class OptimizerParams;

class Optimizer
{
public:
    virtual ~Optimizer() = default;

    virtual OptimizerResult optimize() = 0;

    virtual const OptimizerParams &param() const = 0;

};

} // namespace navapex::traj_opt
