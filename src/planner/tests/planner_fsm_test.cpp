#include <planner/planner.hpp>
#include <perception/maps/grid_map_2d.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>

using namespace navapex;
using planner::LocalPlanInput;
using planner::LocalPlanResult;
using planner::Planner;

namespace
{
constexpr double pi = 3.14159265358979323846;
std::ofstream trace;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

auto makeMap(bool blocked = false)
{
    return std::make_shared<perception::GridMap2D>(
        perception::GridGeometry2D{0.1, 100, 100, -5, -5},
        std::vector<std::uint8_t>(10000, blocked ? perception::Lethal : perception::Free));
}

void setYaw(LocalPlanInput &input, double yaw)
{
    input.state.pose.orientation = {std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)};
}

LocalPlanInput inputAt(double x = 0, double yaw = 0)
{
    LocalPlanInput input;
    input.map = makeMap();
    input.state.pose.position.x = x;
    setYaw(input, yaw);
    input.state.stamp = 1;
    return input;
}

void plan(Planner &planner, const LocalPlanInput &input, double goal_yaw = 0)
{
    planner::GlobalPlanInput global;
    global.map = input.map;
    global.start = {0, 0, toPose2D(input.state.pose).yaw};
    global.goal = {2, 0, goal_yaw};
    auto result = planner.planGlobal(global);
    require(result.success, "global plan failed");
    if (trace.is_open())
    {
        trace << "global " << result.success << ' ' << result.path.size() << '\n';
        for (const auto &point : result.path) trace << point.transpose() << '\n';
    }
}

LocalPlanResult tick(Planner &planner, LocalPlanInput &input, double dt = 0.1)
{
    input.state.stamp += dt;
    auto result = planner.planLocal(input);
    if (trace.is_open())
        trace << "local " << input.state.stamp << ' ' << result.success << ' ' << result.goal_reached << ' '
              << result.velocity << ' ' << result.angular_velocity << ' ' << result.local_goal.transpose() << '\n'
              << result.initial_trajectory << '\n' << result.trajectory << '\n';
    return result;
}

void reusableSearcher()
{
    auto input = std::make_shared<path_plan::SearchInput>();
    path_plan::SearcherInterface searcher("astar2d", input);
    auto check = [&]()
    {
        auto result = searcher.search();
        require(result.success && !result.path.empty(), "reused searcher must find a path");
        Eigen::Vector3d start(input->start.x, input->start.y, input->start.z);
        Eigen::Vector3d goal(input->goal.x, input->goal.y, input->goal.z);
        require((result.path.front() - start).norm() < 1e-9 &&
                (result.path.back() - goal).norm() < 1e-9,
                "reused searcher must use the updated endpoints");
    };
    auto original = makeMap();
    *input = {{0, 0, 0}, {2, 0, 0}, original, 0.125};
    check();

    auto shifted = std::make_shared<perception::GridMap2D>(
        perception::GridGeometry2D{0.2, 20, 12, 4, -2},
        std::vector<std::uint8_t>(240, perception::Free));
    *input = {{4.5, -1.5, 0}, {6.5, -0.5, 0}, shifted, 0};
    check();

    std::vector<std::uint8_t> costs(60, perception::Lethal);
    for (int x = 0; x < 20; ++x) costs[20 + x] = perception::Free;
    auto corridor = std::make_shared<perception::GridMap2D>(
        perception::GridGeometry2D{0.1, 20, 3, 0, 0}, std::move(costs));
    *input = {{0.15, 0.15, 0}, {1.85, 0.15, 0}, corridor, 0};
    check();
    input->radius = 0.125;
    require(!searcher.search().success, "updated radius must reject the narrow corridor");
    input->radius = 0;
    check();
    input->map.reset();
    require(!searcher.search().success, "reused searcher must reject a null map");

    *input = {{2, 1, 0}, {-1, -1, 0}, original, 0.125};
    check();
}

void startAlignment()
{
    Planner planner(robot_model::RobotModel::create());
    require(planner.param().type == "diff", "planner must load its type from planner.json");
    auto input = inputAt(0, pi / 2);
    plan(planner, input);
    input.state.twist.linear.x = 0.18;
    input.state.twist.angular.z = 0.3;
    auto result = tick(planner, input);
    require(result.success && result.trajectory.getPieceNum() == 0, "start must brake before planning");
    require(std::abs(result.velocity - 0.14) < 1e-9 && result.angular_velocity >= 0,
            "moving start must decelerate before rotating");
    input.state.twist = {};
    tick(planner, input);
    result = tick(planner, input, 0.3);
    require(result.success && result.velocity == 0 && result.angular_velocity < 0,
            "start rotation must turn toward path with zero linear speed");
    require(result.trajectory.getPieceNum() == 0 && result.initial_trajectory.getPieceNum() == 0,
            "rotation must not invoke trajectory optimization");
    require(std::abs(result.angular_velocity + 0.2) < 1e-9, "start rotation must limit speed change to 0.2 rad/s per cycle");
    setYaw(input, 0.3);
    input.state.twist.angular.z = -0.8;
    result = tick(planner, input);
    require(std::abs(result.angular_velocity + 0.6) < 1e-9, "start rotation must limit deceleration symmetrically");
    setYaw(input, pi / 2);
    input.state.twist.angular.z = 0;
    input.state.twist.linear.x = 0.01;
    result = tick(planner, input);
    require(result.velocity == 0, "rotation must keep linear command exactly zero");
    input.state.twist = {};
    setYaw(input, 0.3); // Within 20 degrees, outside final yaw tolerance.
    result = tick(planner, input);
    require(result.angular_velocity == 0, "start tolerance must stop pursuing exact path yaw");
    result = tick(planner, input, 0.3);
    require(result.success && result.trajectory.getPieceNum() > 0 && !result.goal_reached,
            "start must hand off to navigation inside 20 degrees");

    planner.reset();
    setYaw(input, 0.8); // Below 60 degrees: skip initial rotation.
    plan(planner, input);
    result = tick(planner, input);
    require(result.success && result.trajectory.getPieceNum() > 0, "moderate heading error must navigate directly");
}

void goalAlignment()
{
    Planner planner(robot_model::RobotModel::create());
    auto input = inputAt(2, 0.3);
    plan(planner, input);
    tick(planner, input);
    auto result = tick(planner, input, 0.3);
    require(result.success && result.velocity == 0 && result.angular_velocity < 0 && !result.goal_reached,
            "goal must continue aligning outside final yaw tolerance");
    require(std::abs(result.angular_velocity + 0.2) < 1e-9, "goal rotation must limit speed increase to 0.2 rad/s per cycle");
    setYaw(input, 0.2);
    input.state.twist.angular.z = -0.4;
    result = tick(planner, input);
    require(std::abs(result.angular_velocity + 0.16) < 1e-9, "goal must use gain 0.8 and allow prompt deceleration");
    setYaw(input, 0.05);
    result = tick(planner, input);
    require(!result.goal_reached && result.angular_velocity == 0, "goal must command zero immediately inside yaw tolerance");
    input.state.twist = {};
    tick(planner, input);
    result = tick(planner, input, 0.3);
    require(result.success && result.goal_reached && result.velocity == 0 && result.angular_velocity == 0,
            "aligned and stopped goal must finish");
    require(tick(planner, input).goal_reached, "finished state must persist");

    // Arrival position drift must return to trajectory navigation.
    planner.reset();
    setYaw(input, 0);
    plan(planner, input, 1);
    tick(planner, input);
    tick(planner, input, 0.3);
    input.state.pose.position.x = 1.7;
    result = tick(planner, input);
    require(result.success && result.trajectory.getPieceNum() > 0 && !result.goal_reached,
            "leaving position tolerance must resume navigation");

    // Angle wrapping must choose the short turn across +/- pi.
    planner.reset();
    input.state.pose.position.x = 2;
    setYaw(input, pi - 0.2);
    plan(planner, input, -pi + 0.2);
    tick(planner, input);
    result = tick(planner, input, 0.3);
    require(result.angular_velocity > 0, "goal must rotate by the wrapped shortest angle");
}

void failureRecovery()
{
    Planner planner(robot_model::RobotModel::create());
    auto input = inputAt(0, pi / 2);
    plan(planner, input);
    tick(planner, input);
    tick(planner, input, 0.3);
    auto free = input.map;
    input.map = makeMap(true);
    for (int i = 0; i < 2; ++i)
    {
        auto result = tick(planner, input);
        require(!result.success && result.velocity == 0 && result.angular_velocity == 0,
                "colliding rotation must fail with zero command");
    }
    input.map = free;
    auto result = tick(planner, input);
    require(result.success && result.angular_velocity < 0 && result.trajectory.getPieceNum() == 0,
            "rotation must resume after collision clears");
    input.map.reset();
    require(!tick(planner, input).success && !tick(planner, input).success, "invalid input must remain failed");
    input.map = free;
    require(tick(planner, input).angular_velocity < 0, "repeated invalid input must preserve rotation stage");

    planner.reset();
    setYaw(input, 0);
    plan(planner, input);
    input.map = makeMap(true);
    require(!tick(planner, input).success, "blocked navigation must fail");
    input.map = free;
    result = tick(planner, input);
    require(result.success && result.trajectory.getPieceNum() > 0, "navigation must retry after obstacles clear");

    plan(planner, input);
    planner.discardWarmStart();
    result = tick(planner, input);
    require(result.success && result.trajectory.getPieceNum() > 0,
            "global replanning and forced local search must reuse the same searcher");
}

void closedLoop()
{
    Planner planner(robot_model::RobotModel::create());
    auto input = inputAt(0, pi / 2);
    plan(planner, input, -pi / 2);
    for (int i = 0; i < 700; ++i)
    {
        auto result = tick(planner, input);
        require(result.success, "closed loop planning failed");
        require(result.velocity >= 0 && result.velocity <= planner.param().max_velocity &&
                std::abs(result.angular_velocity) <= planner.param().max_angular_velocity,
                "closed loop command exceeded velocity limits");
        if (result.goal_reached)
        {
            auto pose = toPose2D(input.state.pose);
            require(std::hypot(pose.x - 2, pose.y) <= planner.param().position_tolerance,
                    "closed loop finished outside position tolerance");
            require(std::abs(std::remainder(pose.yaw + pi / 2, 2 * pi)) <= planner.param().yaw_tolerance,
                    "closed loop finished outside yaw tolerance");
            std::cout << "closed loop finished after " << i + 1 << " steps\n";
            return;
        }
        double yaw = toPose2D(input.state.pose).yaw;
        double dt = planner.param().control_period;
        input.state.pose.position.x += result.velocity * std::cos(yaw) * dt;
        input.state.pose.position.y += result.velocity * std::sin(yaw) * dt;
        setYaw(input, yaw + result.angular_velocity * dt);
        input.state.acceleration.linear.x = (result.velocity - input.state.twist.linear.x) / dt;
        input.state.acceleration.linear.y = result.velocity * result.angular_velocity;
        input.state.acceleration.angular.z = (result.angular_velocity - input.state.twist.angular.z) / dt;
        input.state.twist.linear.x = result.velocity;
        input.state.twist.angular.z = result.angular_velocity;
    }
    throw std::runtime_error("closed loop did not finish");
}

void recoveryAcrossPhases()
{
    Planner planner(robot_model::RobotModel::create());
    auto input = inputAt(0, pi / 2);
    plan(planner, input);
    auto free = input.map;
    input.map.reset();
    require(!tick(planner, input).success && !tick(planner, input).success,
            "new path must tolerate repeated invalid input");
    input.map = free;
    tick(planner, input);
    auto result = tick(planner, input, 0.3);
    require(result.success && result.velocity == 0 && result.angular_velocity < 0,
            "new path recovery must still enter initial rotation");

    // Exercise the map cache during rotation, then use it for trajectory optimization.
    input.map = makeMap(true);
    require(!tick(planner, input).success, "new blocked map must invalidate the rotation ESDF");
    input.map = makeMap();
    setYaw(input, 0);
    tick(planner, input);
    result = tick(planner, input, 0.3);
    require(result.success && result.trajectory.getPieceNum() > 0,
            "rotation must hand off with the latest map after recovery");
    require(tick(planner, input).success, "navigation must reuse its warm start");
    input.map.reset();
    require(!tick(planner, input).success && !tick(planner, input).success,
            "navigation must reject repeated invalid input");
    input.map = free;
    result = tick(planner, input);
    require(result.success && result.trajectory.getPieceNum() > 0,
            "navigation must recover using a fresh search");

    input.state.pose.position.x = 2;
    setYaw(input, 0.4);
    tick(planner, input);
    tick(planner, input, 0.3);
    input.map.reset();
    require(!tick(planner, input).success && !tick(planner, input).success,
            "goal rotation must reject repeated invalid input");
    input.map = free;
    result = tick(planner, input);
    require(result.success && result.velocity == 0 && result.angular_velocity < 0,
            "goal rotation recovery must retain its alignment stage");
    setYaw(input, 0);
    tick(planner, input);
    require(tick(planner, input, 0.3).goal_reached, "goal must finish after recovered rotation");
    input.map.reset();
    require(!tick(planner, input).success && !tick(planner, input).success,
            "finished planner must still reject invalid input");
    input.map = free;
    require(tick(planner, input).goal_reached, "finished phase must survive input failure");

    // Reset and a new global path must clear both failed state and forced-search history.
    planner.reset();
    require(!tick(planner, input).success, "reset must clear the global path");
    input = inputAt();
    plan(planner, input);
    planner.discardWarmStart();
    planner.discardWarmStart();
    require(tick(planner, input).success, "repeated warm-start discard must remain safe");
}

void globalGeometryAndGoalYaw()
{
    Planner planner(robot_model::RobotModel::create());
    auto input = inputAt(0, -pi / 2);
    planner::GlobalPlanInput global{input.map, {0, 0, -pi / 2}, {2, 1, pi / 2}};
    auto path = planner.planGlobal(global);
    require(path.success && !path.path.empty(), "diagonal global plan failed");
    require(path.path == planner.globalPath(), "returned and stored global geometry must agree");
    for (const auto &point : path.path)
        require(point.z() == 0, "global path must contain positions, not per-point yaw");
    require(path.path.front().norm() < 1e-9 &&
            (path.path.back() - Eigen::Vector3d(2, 1, 0)).norm() < 1e-9,
            "global path must preserve the geometric endpoints");

    tick(planner, input);
    auto result = tick(planner, input, 0.3);
    require(result.success && result.velocity == 0 && result.angular_velocity > 0 &&
            result.trajectory.getPieceNum() == 0,
            "initial alignment must derive its direction from diagonal path geometry");

    input.state.pose.position = {2, 1, 0};
    setYaw(input, 0);
    tick(planner, input);
    result = tick(planner, input, 0.3);
    require(result.success && result.velocity == 0 && result.angular_velocity > 0 &&
            result.trajectory.getPieceNum() == 0 && std::abs(result.local_goal.z() - pi / 2) < 1e-9,
            "arrival must align to the task goal yaw instead of the path tangent");
    setYaw(input, pi / 2);
    tick(planner, input);
    require(tick(planner, input, 0.3).goal_reached, "the separately stored goal yaw must finish alignment");
}

void oppositeStartAndReplan()
{
    Planner planner(robot_model::RobotModel::create());
    auto input = inputAt(0, pi);
    plan(planner, input, pi / 2);
    tick(planner, input);
    auto result = tick(planner, input, 0.3);
    require(result.success && result.velocity == 0 && std::abs(result.angular_velocity) > 0 &&
            result.trajectory.getPieceNum() == 0 && result.initial_trajectory.getPieceNum() == 0,
            "an opposite-facing robot must rotate before trajectory optimization");

    planner::GlobalPlanInput global{input.map, {0, 0, pi}, {-2, 0, pi / 2}};
    require(planner.planGlobal(global).success, "reversed global plan failed");
    result = tick(planner, input);
    require(result.success && result.trajectory.getPieceNum() > 0,
            "a new aligned path must clear the previous initial rotation and handle the pi wrap");
}

void stationaryGoal()
{
    Planner planner(robot_model::RobotModel::create());
    // Use an exact grid center so A* returns a single position with no direction.
    double center = -5 + 0.1 / 2 + 50 * 0.1;
    auto input = inputAt(center, 0.7);
    input.state.pose.position.y = center;
    planner::GlobalPlanInput global{input.map, {center, center, 0.7}, {center, center, -1.2}};
    auto path = planner.planGlobal(global);
    require(path.success && path.path.size() == 1 && path.path.front().z() == 0,
            "a stationary goal must keep its single geometric path point");
    tick(planner, input);
    auto result = tick(planner, input, 0.3);
    require(result.success && result.velocity == 0 && result.angular_velocity < 0 &&
            result.trajectory.getPieceNum() == 0 && std::abs(result.local_goal.z() + 1.2) < 1e-9,
            "a directionless path must align directly to the separately stored final yaw");
    setYaw(input, -1.2);
    tick(planner, input);
    require(tick(planner, input, 0.3).goal_reached, "a stationary goal must finish after yaw alignment");
}

void startAlignmentUsesCurrentState()
{
    Planner planner(robot_model::RobotModel::create());
    auto input = inputAt();
    plan(planner, input);
    setYaw(input, pi); // Facing away now, despite the aligned global-plan start.
    auto result = tick(planner, input);
    require(result.success && result.velocity == 0 && result.trajectory.getPieceNum() == 0 &&
            result.initial_trajectory.getPieceNum() == 0,
            "current heading must trigger initial braking even when global start yaw was aligned");
    result = tick(planner, input, 0.3);
    require(result.success && result.velocity == 0 && std::abs(result.angular_velocity) > 0,
            "initial rotation must use the current robot heading");
    setYaw(input, 0);
    tick(planner, input);
    result = tick(planner, input, 0.3);
    require(result.success && result.trajectory.getPieceNum() > 0,
            "initial rotation must hand off when a later state reports alignment");

    input = inputAt(0, pi);
    plan(planner, input);
    setYaw(input, 0); // Already aligned now, despite the opposite global-plan start.
    result = tick(planner, input);
    require(result.success && result.trajectory.getPieceNum() > 0,
            "an aligned current state must navigate without braking for stale global start yaw");
}
} // namespace

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        trace.open(argv[1]);
        require(trace.is_open(), "cannot open behavior trace");
        trace << std::setprecision(17);
    }
    reusableSearcher();
    startAlignment();
    goalAlignment();
    failureRecovery();
    recoveryAcrossPhases();
    closedLoop();
    globalGeometryAndGoalYaw();
    oppositeStartAndReplan();
    stationaryGoal();
    startAlignmentUsesCurrentState();
    std::cout << "planner FSM checks passed\n";
}
