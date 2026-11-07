#pragma once

#include <perception/maps/base_map.hpp>
#include <vector>

namespace navapex::traj_opt
{
enum class LocalSearchResult
{
    Success,
    InitializationError,
    SearchError
};

class LocalAStar
{
public:
    LocalSearchResult search(const perception::OccupancyMap &map,
        Eigen::Vector3d start, Eigen::Vector3d goal, int cells, double timeout_ms,
        std::vector<Eigen::Vector3d> &path);

private:
    struct Node
    {
        double cost = 0;
        int parent = -1;
        unsigned generation = 0;
        bool closed = false;
    };
    std::vector<Node> nodes_;
    unsigned generation_ = 0;
};
} // namespace navapex::traj_opt
