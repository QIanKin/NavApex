#include "replay.hpp"
#include <perception/maps/grid_map_2d.hpp>
#include <zlib.h>
#include <limits>
#include <stdexcept>

namespace navapex::replay
{
namespace
{
std::string base64(const std::string &input)
{
    constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((input.size() + 2) / 3 * 4);
    for (std::size_t i = 0; i < input.size(); i += 3)
    {
        unsigned value = static_cast<unsigned char>(input[i]) << 16;
        if (i + 1 < input.size()) value |= static_cast<unsigned char>(input[i + 1]) << 8;
        if (i + 2 < input.size()) value |= static_cast<unsigned char>(input[i + 2]);
        out += alphabet[(value >> 18) & 63];
        out += alphabet[(value >> 12) & 63];
        out += i + 1 < input.size() ? alphabet[(value >> 6) & 63] : '=';
        out += i + 2 < input.size() ? alphabet[value & 63] : '=';
    }
    return out;
}

std::string compress(const std::vector<std::uint8_t> &data)
{
    if (data.size() > std::numeric_limits<uInt>::max()) throw std::length_error("Map too large");
    z_stream stream{};
    if (deflateInit2(&stream, Z_BEST_SPEED, Z_DEFLATED, 15 + 16, 8, Z_DEFAULT_STRATEGY) != Z_OK)
        throw std::runtime_error("gzip initialization failed");
    struct End { z_stream *s; ~End() { deflateEnd(s); } } end{&stream};
    std::string out(deflateBound(&stream, data.size()), '\0');
    stream.next_in = const_cast<Bytef *>(data.data());
    stream.avail_in = static_cast<uInt>(data.size());
    stream.next_out = reinterpret_cast<Bytef *>(out.data());
    stream.avail_out = static_cast<uInt>(out.size());
    if (deflate(&stream, Z_FINISH) != Z_STREAM_END) throw std::runtime_error("gzip failed");
    out.resize(stream.total_out);
    return out;
}

Json trajectory(const poly_traj::Trajectory<3> &value)
{
    Json pieces = Json::array();
    for (int i = 0; i < value.getPieceNum(); ++i)
    {
        Json coefficients = Json::array();
        const auto &matrix = value[i].getCoeffMat();
        for (int row = 0; row < 3; ++row)
        {
            Json axis = Json::array();
            for (int col = 0; col < 6; ++col) axis.push_back(matrix(row, col));
            coefficients.push_back(std::move(axis));
        }
        pieces.push_back({{"duration", value[i].getDuration()}, {"coefficients", std::move(coefficients)}});
    }
    return {{"axes", {"x", "y", "yaw"}}, {"coefficient_order", "descending_local_seconds"},
            {"pieces", std::move(pieces)}};
}
} // namespace

bool enabled() noexcept { return planner_tools::logEnabled(planner_tools::LogLevel::Info, planner_tools::LogChannel::Replay); }

Json state(const RobotState &value)
{
    auto pose = toPose2D(value.pose);
    const auto &q = value.pose.orientation;
    return {{"stamp", value.stamp}, {"pose", {pose.x, pose.y, pose.yaw}},
            {"z", value.pose.position.z}, {"quaternion_wxyz", {q.w, q.x, q.y, q.z}},
            {"velocity", {value.twist.linear.x, value.twist.linear.y, value.twist.angular.z}},
            {"velocity_frame", "base"}};
}

void event(Json fields) noexcept
{
    if (!enabled()) return;
    try
    {
        fields["v"] = 1;
        // 字段很小；地图和轨迹走下面的专用接口，不在调用线程编码大对象。
        auto text = fields.dump();
        auto bytes = text.size() + 256;
        planner_tools::logReplay([text = std::move(text)] { return text; }, bytes);
    }
    catch (...) { planner_tools::reportLogFailure("Replay event serialization failed"); }
}

void map(Json fields, std::shared_ptr<const perception::OccupancyMap> value) noexcept
{
    if (!enabled()) return;
    try
    {
        auto grid = std::dynamic_pointer_cast<const perception::GridMap2D>(value);
        if (!grid) throw std::invalid_argument("Replay requires GridMap2D");
        fields["v"] = 1;
        fields["event"] = "costmap";
        auto bytes = grid->data().size() + fields.dump().size() + 1024;
        planner_tools::logReplay([fields = std::move(fields), grid = std::move(grid)]() mutable
        {
            const auto &g = grid->geometry();
            fields["width"] = g.width;
            fields["height"] = g.height;
            fields["resolution"] = g.resolution;
            fields["origin"] = {g.origin_x, g.origin_y};
            fields["encoding"] = "gzip+base64";
            fields["layout"] = "row_major_y_x";
            fields["data"] = base64(compress(grid->data()));
            return fields.dump();
        }, bytes);
    }
    catch (...) { planner_tools::reportLogFailure("Replay map submission failed"); }
}

void plan(Json fields, const planner::LocalPlanResult &result,
          const std::vector<Eigen::Vector3d> &global_path) noexcept
{
    if (!enabled()) return;
    try
    {
        auto bytes = 4096 + fields.dump().size() + global_path.size() * sizeof(Eigen::Vector3d) +
            (result.trajectory.getPieceNum() + result.initial_trajectory.getPieceNum()) * 256;
        planner_tools::logReplay([fields = std::move(fields), result, global_path]() mutable
        {
            fields["v"] = 1;
            fields["event"] = "plan";
            fields["success"] = result.success;
            fields["goal_reached"] = result.goal_reached;
            fields["command"] = {result.success ? result.velocity : 0, result.success ? result.angular_velocity : 0};
            fields["local_goal"] = {result.local_goal.x(), result.local_goal.y(), result.local_goal.z()};
            fields["trajectory"] = trajectory(result.trajectory);
            fields["initial_trajectory"] = trajectory(result.initial_trajectory);
            fields["global_path_axes"] = {"x", "y", "z"};
            fields["global_path"] = Json::array();
            for (const auto &p : global_path) fields["global_path"].push_back({p.x(), p.y(), p.z()});
            return fields.dump();
        }, bytes);
    }
    catch (...) { planner_tools::reportLogFailure("Replay plan submission failed"); }
}

void health(double stamp, std::uint64_t clock_epoch) noexcept
{
    auto h = planner_tools::logHealth();
    event({{"event", "health"}, {"stamp", stamp}, {"clock_epoch", clock_epoch}, {"submitted", h.submitted},
           {"written", h.written}, {"dropped", h.dropped}, {"errors", h.errors}, {"pending", h.pending}});
}
} // namespace navapex::replay
