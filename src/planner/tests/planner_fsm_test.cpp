#include <planner/planner.hpp>
#include <perception/maps/grid_map_2d.hpp>
#include <cmath>
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
    require(planner.planGlobal(global).success, "global plan failed");
}

LocalPlanResult tick(Planner &planner, LocalPlanInput &input, double dt = 0.1)
{
    input.state.stamp += dt;
    return planner.planLocal(input);
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
} // namespace

int main()
{
    reusableSearcher();
    startAlignment();
    goalAlignment();
    failureRecovery();
    closedLoop();
    std::cout << "planner FSM checks passed\n";
}
