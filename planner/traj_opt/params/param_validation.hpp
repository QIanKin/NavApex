#pragma once

#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace traj_opt::param_validation
{
inline void nonnegative(std::initializer_list<double> values)
{
    for (double value : values)
    {
        if (!std::isfinite(value) || value < 0)
        {
            throw std::invalid_argument("Weights, radii and tolerances must be finite and nonnegative");
        }
    }
}

inline void positive(std::initializer_list<double> values)
{
    for (double value : values)
    {
        if (!std::isfinite(value) || value <= 0)
        {
            throw std::invalid_argument("Limits, scales and steps must be finite and positive");
        }          
    }
}
} // namespace traj_opt::param_validation
