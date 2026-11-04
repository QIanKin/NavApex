#pragma once

#include <Eigen/Core>
#include <map/occupancy_map.hpp>
#include <memory>

namespace path_plan
{
struct SearchInput
{
    Eigen::Vector3d start = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal = Eigen::Vector3d::Zero();
    std::shared_ptr<const map::OccupancyMap> map;
};
} // namespace path_plan
