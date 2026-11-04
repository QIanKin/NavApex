#pragma once

#include <Eigen/Core>

namespace map
{
struct DistanceQuery2D
{
    double distance = 0;
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
};

class EsdfMap2D
{
public:
    virtual ~EsdfMap2D() = default;
    virtual bool query(const Eigen::Vector2d &position, DistanceQuery2D &result) const = 0;
    virtual double resolution() const = 0;
};
} // namespace map
