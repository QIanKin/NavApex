#include <astar/astar.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <tools/log.hpp>
#include <utility>

namespace path_plan
{
static constexpr char SEARCHER_TAG[] = "searcher";

AStar::AStar(bool is_3d, AStarParams param,
             std::shared_ptr<const SearchInput> input)
    : param_(std::move(param)), input_(std::move(input)), is_3d_(is_3d)
{
    param_.validate();
    if (!input_)
        throw std::invalid_argument("SearchInput must not be null");
}

const AStarParams &AStar::param() const
{
    return param_;
}

bool AStar::initializeSearch()
{
    const auto &input = *input_;
    const auto space = is_3d_ ? map::Space::Spatial : map::Space::Planar;
    if (!input.map || input.map->space() != space || !input.start.allFinite() || !input.goal.allFinite())
    {
        LOGE(SEARCHER_TAG) << "invalid search input or map dimension";
        return false;
    }
    resolution_ = input.map->resolution();
    if (!std::isfinite(resolution_) || resolution_ <= 0 ||
        (!is_3d_ && input.start.z() != input.goal.z()))
    {
        LOGE(SEARCHER_TAG) << "invalid resolution or different planar endpoint heights";
        return false;
    }

    // 栅格以起点对齐，窗口中心靠近起终点中点；转换前检查范围，避免整数溢出。
    const int cells = param_.search.search_cells;
    const Eigen::Vector3d delta = (input.goal - input.start) / resolution_;
    if (!delta.allFinite() || delta.cwiseAbs().maxCoeff() > cells - 1)
    {
        LOGW(SEARCHER_TAG) << "endpoints outside search window cells=" << cells;
        return false;
    }
    const Eigen::Vector3i offset = delta.array().round().cast<int>();
    for (int axis = 0; axis < 3; ++axis)
        first_[axis] = (cells - 1 - offset[axis]) / 2;
    last_ = first_ + offset;
    if (!is_3d_)
        first_.z() = last_.z() = 0;

    if (!segmentFree(input.start, input.start) ||
        !segmentFree(positionOf(last_), input.goal))
    {
        LOGW(SEARCHER_TAG) << "search endpoint or goal connection in collision";
        return false;
    }
    const std::size_t count = static_cast<std::size_t>(cells) * cells *
        (is_3d_ ? cells : 1);
    nodes_.resize(count);
    if (++generation_ == 0)
    {
        for (auto &node : nodes_)
            node.generation = 0;
        ++generation_;
    }
    return true;
}

Eigen::Vector3d AStar::positionOf(const Eigen::Vector3i &index) const
{
    return input_->start + (index - first_).cast<double>() * resolution_;
}

int AStar::address(const Eigen::Vector3i &index) const
{
    const int cells = param_.search.search_cells;
    if (!is_3d_)
        return index.x() * cells + index.y();
    return (index.x() * cells + index.y()) * cells + index.z();
}

Eigen::Vector3i AStar::unpack(int id) const
{
    const int cells = param_.search.search_cells;
    if (!is_3d_)
        return {id / cells, id % cells, 0};
    return {id / (cells * cells), (id / cells) % cells, id % cells};
}

double AStar::heuristic(const Eigen::Vector3i &index) const
{
    const Eigen::Vector3i delta = (last_ - index).cwiseAbs();
    int v[3] = {delta.x(), delta.y(), delta.z()};
    std::sort(v, v + 3);
    return std::sqrt(3.0) * v[0] + std::sqrt(2.0) * (v[1] - v[0]) + v[2] - v[1];
}

bool AStar::segmentFree(const Eigen::Vector3d &start, const Eigen::Vector3d &goal) const
{
    if (!start.allFinite() || !goal.allFinite())
        return false;
    const double distance = ((goal - start) / resolution_).norm();
    if (!std::isfinite(distance))
        return false;
    const int samples = std::max(2, static_cast<int>(std::ceil(distance * 2)));
    for (int i = 0; i <= samples; ++i)
    {
        if (input_->map->occupied(start + (goal - start) * (static_cast<double>(i) / samples)))
            return false;
    }
    return true;
}

bool AStar::transitionFree(const Eigen::Vector3i &index, const Eigen::Vector3i &step) const
{
    // 对角移动要求所有经过的相邻格均为空闲，禁止从障碍物的边或角挤过。
    for (int mask = 1; mask < 8; ++mask)
    {
        Eigen::Vector3i offset = Eigen::Vector3i::Zero();
        bool subset = true;
        for (int axis = 0; axis < 3; ++axis)
        {
            if (mask & (1 << axis))
            {
                subset = subset && step[axis] != 0;
                offset[axis] = step[axis];
            }
        }
        if (subset)
        {
            const Eigen::Vector3d position = positionOf(index + offset);
            if (!position.allFinite() || input_->map->occupied(position))
                return false;
        }
    }
    return segmentFree(positionOf(index), positionOf(index + step));
}

Result AStar::search()
{
    const auto begin = std::chrono::steady_clock::now();
    Result result;
    const auto elapsed = [&]()
    {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    };
    if (!initializeSearch())
    {
        result.elapsed_ms = elapsed();
        return result;
    }

    struct Entry { double score, cost; int id; };
    struct Later
    {
        bool operator()(const Entry &a, const Entry &b) const
        { return a.score > b.score || (a.score == b.score && a.id > b.id); }
    };
    std::priority_queue<Entry, std::vector<Entry>, Later> open;
    const int first_id = address(first_), last_id = address(last_);
    nodes_[first_id] = {0, -1, generation_, false};
    open.push({heuristic(first_), 0, first_id});
    const int cells = param_.search.search_cells;
    while (!open.empty())
    {
        if (elapsed() > param_.search.timeout_ms)
        {
            LOGD(SEARCHER_TAG) << "A* timeout timeout_ms=" << param_.search.timeout_ms;
            break;
        }
        const auto entry = open.top();
        open.pop();
        auto &current = nodes_[entry.id];
        // 堆中保留旧条目，通过入堆时的代价跳过过期记录。
        if (current.closed || entry.cost != current.cost)
            continue;
        if (entry.id == last_id)
        {
            for (int id = last_id; id >= 0; id = nodes_[id].parent)
                result.path.push_back(positionOf(unpack(id)));
            std::reverse(result.path.begin(), result.path.end());
            if ((result.path.back() - input_->goal).squaredNorm() > 0)
                result.path.push_back(input_->goal);
            for (std::size_t i = 1; i < result.path.size(); ++i)
                result.total_cost += (result.path[i] - result.path[i - 1]).norm();
            result.success = true;
            result.elapsed_ms = elapsed();
            LOGI(SEARCHER_TAG) << "A* success points=" << result.path.size()
                               << " elapsed_ms=" << result.elapsed_ms;
            return result;
        }
        current.closed = true;
        const Eigen::Vector3i index = unpack(entry.id);
        const int z_extent = is_3d_ ? 1 : 0;
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -z_extent; dz <= z_extent; ++dz)
                {
                    const int axes = (dx != 0) + (dy != 0) + (dz != 0);
                    if (axes == 0)
                        continue;
                    const Eigen::Vector3i step(dx, dy, dz);
                    const Eigen::Vector3i next = index + step;
                    if ((next.array() < 0).any() || (next.array() >= cells).any())
                        continue;
                    const int id = address(next);
                    auto &node = nodes_[id];
                    const bool visited = node.generation == generation_;
                    const double cost = current.cost + std::sqrt(static_cast<double>(axes));
                    if ((visited && (node.closed || cost >= node.cost)) || !transitionFree(index, step))
                        continue;
                    node = {cost, entry.id, generation_, false};
                    open.push({cost + heuristic(next), cost, id});
                }
    }
    result.elapsed_ms = elapsed();
    LOGD(SEARCHER_TAG) << "A* search failed elapsed_ms=" << result.elapsed_ms;
    return result;
}
} // namespace path_plan
