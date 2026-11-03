#include "robot_model.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_model
{
namespace
{
/// 保证没有缺省尺寸或非法半径进入碰撞查询。
void validateRadius(double radius)
{
    if (!std::isfinite(radius) || radius <= 0)
        throw std::invalid_argument("Robot radius must be finite and positive");
}
} // namespace

CircleModel::CircleModel(double radius) : radius_(radius)
{
    validateRadius(radius_);
}

const char *CircleModel::type() const
{
    return "circle";
}

Space CircleModel::space() const
{
    return Space::Planar;
}

double CircleModel::radius() const
{
    return radius_;
}

SphereModel::SphereModel(double radius) : radius_(radius)
{
    validateRadius(radius_);
}

const char *SphereModel::type() const
{
    return "sphere";
}

Space SphereModel::space() const
{
    return Space::Spatial;
}

double SphereModel::radius() const
{
    return radius_;
}
} // namespace robot_model
