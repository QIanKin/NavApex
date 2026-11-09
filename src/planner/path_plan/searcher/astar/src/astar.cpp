#include <astar/astar.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <tools/log.hpp>
#include <utility>

namespace navapex::path_plan
{
static constexpr char SEARCHER_TAG[] = "searcher";

AStar::AStar(bool is_3d, AStarParams param,
             std::shared_ptr<const SearchInput> input)
            : param_(std::move(param)), input_(std::move(input)), is_3d_(is_3d)
{
    
}

const AStarParams &AStar::param() const
{
    return param_;
}

bool AStar::initializeSearch()
{
    const auto &input = *input_;
    Eigen::Vector3d start(input.start.x, input.start.y, input.start.z);
    Eigen::Vector3d goal(input.goal.x, input.goal.y, input.goal.z);
    auto space = is_3d_ ? perception::Space::Spatial : perception::Space::Planar;
    if (!input.map || input.map->space() != space || !start.allFinite() || !goal.allFinite())
    {
        LOGE(SEARCHER_TAG) << "invalid search input or map dimension";
        return false;
    }
    resolution_ = input.map->resolution();
    if (!std::isfinite(resolution_) || resolution_ <= 0 ||
        !std::isfinite(input.radius) || input.radius < 0 ||
        (input.ignore_goal_obstacles &&
         (!std::isfinite(input.goal_ignore_radius) || input.goal_ignore_radius <= 0)) ||
        (!is_3d_ && start.z() != goal.z()))
    {
        LOGE(SEARCHER_TAG) << "invalid resolution, radius, goal ignore radius or different planar endpoint heights";
        return false;
    }

    Eigen::Vector3d lower, upper;
    input.map->bounds(lower, upper);
    Eigen::Vector3d extent = (upper - lower) / resolution_;
    int axes = is_3d_ ? 3 : 2;
    if (!lower.allFinite() || !upper.allFinite() || !extent.allFinite())
        return false;
    for (int axis = 0; axis < axes; ++axis)
    {
        if (extent[axis] < 1 || extent[axis] > param_.max_nodes)
            return false;
        size_[axis] = static_cast<int>(std::round(extent[axis]));
        double first = (start[axis] - lower[axis]) / resolution_;
        double last = (goal[axis] - lower[axis]) / resolution_;
        if (first < 0 || last < 0 || first >= size_[axis] || last >= size_[axis])
            return false;
        first_[axis] = static_cast<int>(std::floor(first));
        last_[axis] = static_cast<int>(std::floor(last));
    }
    origin_ = lower + Eigen::Vector3d::Constant(0.5 * resolution_);
    if (!is_3d_)
    {
        size_.z() = 1;
        first_.z() = last_.z() = 0;
        origin_.z() = start.z();
    }

    // 实际端点可能可通行，但所在格的格心碰撞；只在相邻格内寻找可直连的格心。
    auto connectEndpoint = [&](const Eigen::Vector3d &point, Eigen::Vector3i &index, bool allow_goal_relaxation)
    {
        if (segmentFree(point, positionOf(index), allow_goal_relaxation)) return true;
        if (!footprintFree(point, allow_goal_relaxation)) return false;
        Eigen::Vector3i original = index;
        double best = std::numeric_limits<double>::infinity();
        for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y)
                for (int z = is_3d_ ? -1 : 0; z <= (is_3d_ ? 1 : 0); ++z)
                {
                    Eigen::Vector3i candidate = original + Eigen::Vector3i(x, y, z);
                    if ((candidate.array() < 0).any() || (candidate.array() >= size_.array()).any()) continue;
                    double distance = (positionOf(candidate) - point).squaredNorm();
                    if (distance < best && segmentFree(point, positionOf(candidate), allow_goal_relaxation))
                    {
                        best = distance;
                        index = candidate;
                    }
                }
        return std::isfinite(best);
    };
    if (!connectEndpoint(start, first_, false) || !connectEndpoint(goal, last_, true))
    {
        LOGW(SEARCHER_TAG) << "search endpoint or goal connection in collision";
        return false;
    }
    std::size_t count = 1;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (static_cast<std::size_t>(size_[axis]) > param_.max_nodes / count)
        {
            LOGW(SEARCHER_TAG) << "search grid exceeds node budget";
            return false;
        }
        count *= size_[axis];
    }
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
    return origin_ + index.cast<double>() * resolution_;
}

int AStar::address(const Eigen::Vector3i &index) const
{
    return (index.x() * size_.y() + index.y()) * size_.z() + index.z();
}

Eigen::Vector3i AStar::unpack(int id) const
{
    return {id / (size_.y() * size_.z()), (id / size_.z()) % size_.y(), id % size_.z()};
}

double AStar::heuristic(const Eigen::Vector3i &index) const
{
    Eigen::Vector3i delta = (last_ - index).cwiseAbs();
    int v[3] = {delta.x(), delta.y(), delta.z()};
    std::sort(v, v + 3);
    return std::sqrt(3.0) * v[0] + std::sqrt(2.0) * (v[1] - v[0]) + v[2] - v[1];
}

bool AStar::footprintFree(const Eigen::Vector3d &position, bool allow_goal_relaxation) const
{
    Eigen::Vector3d goal(input_->goal.x, input_->goal.y, input_->goal.z);
    bool ignore_obstacles = allow_goal_relaxation && input_->ignore_goal_obstacles &&
        (position - goal).squaredNorm() <= input_->goal_ignore_radius * input_->goal_ignore_radius;
    int radius = static_cast<int>(std::ceil(input_->radius / resolution_));
    int z_extent = is_3d_ ? radius : 0;
    // 半径向上取整到格，扫描圆形（二维）或球形（三维）范围内的格点。
    for (int dx = -radius; dx <= radius; ++dx)
        for (int dy = -radius; dy <= radius; ++dy)
            for (int dz = -z_extent; dz <= z_extent; ++dz)
            {
                if (dx * dx + dy * dy + dz * dz > radius * radius) continue;
                Eigen::Vector3d sample = position + resolution_ * Eigen::Vector3d(dx, dy, dz);
                if (ignore_obstacles)
                {
                    // 只放宽终点附近的占据限制，不允许车身越出地图。
                    Eigen::Vector3d cell = (sample - origin_) / resolution_ + Eigen::Vector3d::Constant(0.5);
                    for (int axis = 0; axis < (is_3d_ ? 3 : 2); ++axis)
                        if (cell[axis] < 0 || cell[axis] >= size_[axis]) return false;
                    continue;
                }
                if (input_->map->occupied({sample.x(), sample.y(), sample.z()})) return false;
            }
    return true;
}

bool AStar::segmentFree(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                        bool allow_goal_relaxation) const
{
    double distance = ((goal - start) / resolution_).norm();
    int samples = std::max(2, static_cast<int>(std::ceil(distance * 2)));
    for (int i = 0; i <= samples; ++i)
    {
        Eigen::Vector3d position = start + (goal - start) * (static_cast<double>(i) / samples);
        if (!footprintFree(position, allow_goal_relaxation))
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
            Eigen::Vector3d position = positionOf(index + offset);
            if (!footprintFree(position))
                return false;
        }
    }
    return segmentFree(positionOf(index), positionOf(index + step));
}

SearchResult AStar::search()
{
    auto begin = std::chrono::steady_clock::now();
    SearchResult result;
    auto elapsed = [&]()
    {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    };
    if (!initializeSearch())
    {
        LOGW(SEARCHER_TAG) << "A* initialization failed elapsed_ms=" << elapsed();
        return result;
    }

    struct Entry { double score, cost; int id; };
    struct Later
    {
        bool operator()(const Entry &a, const Entry &b) const
        { return a.score > b.score || (a.score == b.score && a.id > b.id); }
    };
    std::priority_queue<Entry, std::vector<Entry>, Later> open;
    int first_id = address(first_), last_id = address(last_);
    nodes_[first_id] = {0, -1, generation_, false};
    open.push({heuristic(first_), 0, first_id});
    while (!open.empty())
    {
        if (elapsed() > param_.timeout_ms)
        {
            LOGD(SEARCHER_TAG) << "A* timeout timeout_ms=" << param_.timeout_ms;
            break;
        }
        auto entry = open.top();
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
            Eigen::Vector3d start(input_->start.x, input_->start.y, input_->start.z);
            Eigen::Vector3d goal(input_->goal.x, input_->goal.y, input_->goal.z);
            if (result.path.front() != start)
                result.path.insert(result.path.begin(), start);
            if ((result.path.back() - goal).squaredNorm() > 0)
                result.path.push_back(goal);
            result.success = true;
            LOGI(SEARCHER_TAG) << "A* success points=" << result.path.size()
                               << " elapsed_ms=" << elapsed();
            return result;
        }
        current.closed = true;
        Eigen::Vector3i index = unpack(entry.id);
        int z_extent = is_3d_ ? 1 : 0;
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -z_extent; dz <= z_extent; ++dz)
                {
                    int axes = (dx != 0) + (dy != 0) + (dz != 0);
                    if (axes == 0)
                        continue;
                    Eigen::Vector3i step(dx, dy, dz);
                    Eigen::Vector3i next = index + step;
                    if ((next.array() < 0).any() || (next.array() >= size_.array()).any())
                        continue;
                    int id = address(next);
                    auto &node = nodes_[id];
                    bool visited = node.generation == generation_;
                    double cost = current.cost + std::sqrt(static_cast<double>(axes));
                    if ((visited && (node.closed || cost >= node.cost)) || !transitionFree(index, step))
                        continue;
                    node = {cost, entry.id, generation_, false};
                    open.push({cost + heuristic(next), cost, id});
                }
    }
    LOGD(SEARCHER_TAG) << "A* search failed elapsed_ms=" << elapsed();
    return result;
}
} // namespace navapex::path_plan
