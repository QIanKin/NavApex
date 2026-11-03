#pragma once

#include <robot_model/robot_model.hpp>

namespace map
{

class OccupancyMap
{
public:
    virtual ~OccupancyMap() = default;
    virtual bool occupied(const Eigen::Vector3d &position) const = 0;
    virtual double resolution() const = 0; // 栅格边长，单位米；由实现方保证有限且大于零
    virtual robot_model::Space space() const = 0;
};
} // namespace map
