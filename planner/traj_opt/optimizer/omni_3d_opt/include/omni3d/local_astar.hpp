#pragma once

#include <map/occupancy_map.hpp>
#include <vector>

namespace omni3d
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
    LocalSearchResult search(const map::OccupancyMap &map,
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
} // namespace omni3d
