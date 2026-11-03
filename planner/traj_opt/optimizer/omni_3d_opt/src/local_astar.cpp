// Adapted from df_a path_searching/dyn_a_star.cpp; see repository LICENSE.
#include <omni3d/local_astar.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <tools/log.hpp>

namespace omni3d
{
static constexpr char OPTIMIZER_TAG[] = "optimizer";

LocalSearchResult LocalAStar::search(const map::OccupancyMap &map,
                                     Eigen::Vector3d start, 
                                     Eigen::Vector3d goal, 
                                     int cells, 
                                     double timeout_ms,
                                     std::vector<Eigen::Vector3d> &path)
{
    path.clear();
    const auto begin = std::chrono::steady_clock::now();
    const double resolution = map.resolution();
    if (cells < 5 || cells > 200 || !std::isfinite(timeout_ms) || timeout_ms <= 0 ||
        !start.allFinite() || !goal.allFinite())
    {
        LOGE(OPTIMIZER_TAG) << "invalid local search input";
        return LocalSearchResult::InitializationError;
    }
    const Eigen::Vector3d center = (start + goal) * 0.5;
    const auto indexOf = [&](const Eigen::Vector3d &p, Eigen::Vector3i &index)
    {
        const Eigen::Vector3d local = (p - center) / resolution;
        if (!local.allFinite() || local.cwiseAbs().maxCoeff() > cells)
            return false;
        index = (local + Eigen::Vector3d::Constant(0.5)).cast<int>() +
            Eigen::Vector3i::Constant(cells / 2);
        return (index.array() >= 1).all() && (index.array() < cells - 1).all();
    };
    const auto positionOf = [&](const Eigen::Vector3i &i) -> Eigen::Vector3d
    {
        return center + (i - Eigen::Vector3i::Constant(cells / 2)).cast<double>() * resolution;
    };
    Eigen::Vector3i first, last;
    const auto adjust = [&](Eigen::Vector3d &p, const Eigen::Vector3d &other, Eigen::Vector3i &index)
    {
        if (!indexOf(p, index))
            return false;
        while (map.occupied(positionOf(index)))
        {
            const Eigen::Vector3d delta = p - other;
            if (delta.norm() < 1e-9)
                return false;
            p += delta.normalized() * resolution;
            if (!indexOf(p, index))
                return false;
        }
        return true;
    };
    if (!adjust(start, goal, first) || !adjust(goal, start, last))
    {
        LOGD(OPTIMIZER_TAG) << "local search endpoints unavailable start="
                            << start.transpose() << " goal=" << goal.transpose();
        return LocalSearchResult::InitializationError;
    }
    const auto address = [cells](const Eigen::Vector3i &i)
    { return (i.x() * cells + i.y()) * cells + i.z(); };
    const auto unpack = [cells](int id) -> Eigen::Vector3i
    { return {id / (cells * cells), (id / cells) % cells, id % cells}; };
    const auto heuristic = [&](const Eigen::Vector3i &i)
    {
        Eigen::Vector3i d = (last - i).cwiseAbs();
        int v[3] = {d.x(), d.y(), d.z()};
        std::sort(v, v + 3);
        return 1.0001 * (std::sqrt(3.0) * v[0] + std::sqrt(2.0) * (v[1] - v[0]) + v[2] - v[1]);
    };
    nodes_.resize(static_cast<std::size_t>(cells) * cells * cells);
    if (++generation_ == 0)
    {
        for (auto &node : nodes_) node.generation = 0;
        ++generation_;
    }
    struct Entry { double score, cost; int id; };
    struct Later
    {
        bool operator()(const Entry &a, const Entry &b) const
        { return a.score > b.score || (a.score == b.score && a.id > b.id); }
    };
    std::priority_queue<Entry, std::vector<Entry>, Later> open;
    const int first_id = address(first), last_id = address(last);
    nodes_[first_id] = {0, -1, generation_, false};
    open.push({heuristic(first), 0, first_id});
    while (!open.empty())
    {
        if (std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - begin).count() > timeout_ms)
        {
            LOGD(OPTIMIZER_TAG) << "local search timeout timeout_ms=" << timeout_ms;
            return LocalSearchResult::SearchError;
        }
        const auto entry = open.top();
        open.pop();
        auto &current = nodes_[entry.id];
        if (current.closed || entry.cost != current.cost)
            continue;
        if (entry.id == last_id)
        {
            for (int id = last_id; id >= 0; id = nodes_[id].parent)
                path.push_back(positionOf(unpack(id)));
            std::reverse(path.begin(), path.end());
            return LocalSearchResult::Success;
        }
        current.closed = true;
        const Eigen::Vector3i index = unpack(entry.id);
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if ((dx == 0 && dy == 0 && dz == 0) ||
                        (map.space() == robot_model::Space::Planar && dz != 0))
                        continue;
                    const Eigen::Vector3i next = index + Eigen::Vector3i(dx, dy, dz);
                    if ((next.array() < 1).any() || (next.array() >= cells - 1).any())
                        continue;
                    const int id = address(next);
                    auto &node = nodes_[id];
                    const bool visited = node.generation == generation_;
                    if ((visited && node.closed) || map.occupied(positionOf(next)))
                        continue;
                    const double cost = current.cost + std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (!visited || cost < node.cost)
                    {
                        node = {cost, entry.id, generation_, false};
                        open.push({cost + heuristic(next), cost, id});
                    }
                }
    }
    LOGD(OPTIMIZER_TAG) << "local search no path";
    return LocalSearchResult::SearchError;
}
} // namespace omni3d
