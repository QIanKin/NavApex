#pragma once

#include <planner/path_plan/params/astar_params.hpp>
#include <planner/path_plan/search_input.hpp>
#include <planner/path_plan/searcher/base_searcher.hpp>

namespace path_plan
{
class AStar final : public Searcher
{
public:
    AStar(bool is_3d, AStarParams param, std::shared_ptr<const SearchInput> input);
    AStar(const AStar &) = delete;
    AStar &operator=(const AStar &) = delete;

    Result search() override;

    const AStarParams &param() const override;

private:
    struct Node
    {
        double cost = 0;
        int parent = -1;
        unsigned generation = 0;
        bool closed = false;
    };

    bool initializeSearch();

    Eigen::Vector3d positionOf(const Eigen::Vector3i &index) const;

    int address(const Eigen::Vector3i &index) const;

    Eigen::Vector3i unpack(int id) const;

    double heuristic(const Eigen::Vector3i &index) const;

    bool segmentFree(const Eigen::Vector3d &start, const Eigen::Vector3d &goal) const;

    bool transitionFree(const Eigen::Vector3i &index, const Eigen::Vector3i &step) const;

    AStarParams param_;
    std::shared_ptr<const SearchInput> input_;
    bool is_3d_;
    double resolution_ = 0;
    Eigen::Vector3i first_ = Eigen::Vector3i::Zero();
    Eigen::Vector3i last_ = Eigen::Vector3i::Zero();
    std::vector<Node> nodes_;
    unsigned generation_ = 0;
};
} // namespace path_plan
