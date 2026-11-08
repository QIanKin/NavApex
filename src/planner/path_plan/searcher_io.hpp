#pragma once

#include <perception/maps/base_map.hpp>
#include <memory>
#include <vector>

namespace navapex::path_plan
{
struct SearchInput
{
    Position3D start;
    Position3D goal;
    std::shared_ptr<const perception::OccupancyMap> map;
    double radius = 0;
};

struct SearchResult
{
    bool success = false;
    std::vector<Eigen::Vector3d> path;
};

} // namespace navapex::path_plan
