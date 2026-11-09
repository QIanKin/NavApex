#include <tools/replay.hpp>
#include <perception/maps/grid_map_2d.hpp>
#include <future>
#include <iostream>
#include <thread>

using namespace navapex;

int main(int argc, char **argv)
{
    planner_tools::LogConfig config;
    config.directory = argc > 1 ? argv[1] : "replay-fixture";
    config.name = "replay_fixture";
    config.max_file_bytes = 24 * 1024;
    config.max_files = 32;
    planner_tools::initLog(config);
    planner_tools::setReplayMetadata(replay::Json({{"v", 1}, {"frame", "map"},
        {"source_clock", "synthetic"}, {"robot", {{"shape", "circle"}, {"radius", 0.125}}}}).dump());
    std::promise<std::thread::id> thread;
    planner_tools::logReplay([&thread] {
        thread.set_value(std::this_thread::get_id());
        return "{\"v\":1,\"event\":\"health\",\"stamp\":0,\"dropped\":0}";
    }, 128);
    if (thread.get_future().get() == std::this_thread::get_id()) return 1;

    perception::GridGeometry2D geometry;
    geometry.width = 200; geometry.height = 120; geometry.resolution = 0.02;
    geometry.origin_x = -0.5; geometry.origin_y = -1.2;
    std::vector<std::uint8_t> costs(geometry.width * geometry.height, 0);
    for (int y = 0; y < geometry.height; ++y)
        for (int x = 0; x < geometry.width; ++x)
        {
            double wx = geometry.origin_x + (x + 0.5) * geometry.resolution;
            double wy = geometry.origin_y + (y + 0.5) * geometry.resolution;
            if (wx >= 0.5 && wx <= 2.5 && std::abs(wy) >= 0.175) costs[y * geometry.width + x] = 254;
        }
    // Lossless 0..255 round-trip sentinel, outside the demonstration corridor.
    for (int i = 0; i < 256; ++i) costs[i] = static_cast<std::uint8_t>(i);
    auto map = std::make_shared<const perception::GridMap2D>(geometry, costs);
    replay::map({{"map_id", "global"}, {"layer", "global"}, {"stamp", 0}}, map);
    for (int i = 0; i < 60; ++i)
    {
        RobotState state;
        state.stamp = 10 + i * 0.1;
        state.pose.position.x = 0.1 + i * 0.035;
        state.twist.linear.x = 0.35;
        replay::event({{"event", "pose"}, {"stamp", state.stamp}, {"state_id", i + 1}, {"state", replay::state(state)}});
        replay::map({{"map_id", i + 1}, {"layer", "local"}, {"stamp", state.stamp},
            {"state_id", i + 1}, {"state", replay::state(state)}, {"success", true}}, map);
        planner::LocalPlanResult result;
        result.success = i != 25;
        result.velocity = result.success ? 0.35 : 0;
        Eigen::Matrix<double, 3, 6> coefficients = Eigen::Matrix<double, 3, 6>::Zero();
        coefficients(0, 4) = 0.35;
        coefficients(0, 5) = state.pose.position.x;
        result.trajectory.emplace_back(1.0, coefficients);
        replay::plan({{"stamp", state.stamp}, {"plan_id", i + 1}, {"state_id", i + 1}, {"map_id", i + 1},
            {"goal_id", 1}, {"goal", {2.8, 0, 0}}, {"state", replay::state(state)}, {"elapsed_ms", 2.5},
            {"accepted", i != 40}, {"status", i == 25 ? "planning_failed" : i == 40 ? "discarded_goal_changed" : "planning_success"}},
            result, {{0.1, 0, 0}, {2.8, 0, 0}});
        LOGD("fixture") << "frame=" << i + 1 << " escaped=\"quote\" path=C:\\logs\nsecond line";
        // Keep the fixture deterministic even on very slow machines.
        planner_tools::flushLog();
    }
    planner_tools::flushLog();
    replay::health(16.0);
    planner_tools::flushLog();
    auto health = planner_tools::logHealth();
    if (health.dropped || health.errors || health.pending) return 2;
    // Queue and encoder failures must be counted and must not kill the logger.
    planner_tools::logReplay([] { return std::string(); }, config.max_pending_bytes + 1);
    planner_tools::logReplay([]() -> std::string { throw std::runtime_error("expected test failure"); }, 32);
    planner_tools::flushLog();
    health = planner_tools::logHealth();
    if (health.dropped != 2 || health.errors != 1 || health.pending) return 3;
    std::cout << planner_tools::logFile() << '\n';
    return 0;
}
