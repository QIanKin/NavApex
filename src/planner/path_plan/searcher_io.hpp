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
    bool ignore_goal_obstacles = false;
    double goal_ignore_radius = 0.2;
};

struct SearchResult
{
    bool success = false;
    std::vector<Eigen::Vector3d> path;
};

} // namespace navapex::path_plan
