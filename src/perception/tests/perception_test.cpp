#include <perception/perception_interface.hpp>
#include <perception/params/lidar_2d_params.hpp>
#include <iostream>
#include <stdexcept>

using namespace navapex;

namespace
{
void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}
} // namespace

int main()
{
    perception::PerceptionInterface interface("lidar2d");
    const auto &param = dynamic_cast<const perception::Lidar2DParams &>(interface.param());
    auto small_robot = std::make_shared<robot_model::CircleModel>(0.1);
    auto large_robot = std::make_shared<robot_model::CircleModel>(0.8);
    auto scan = std::make_shared<perception::LaserScan2D>();
    scan->range_max = 1.0;
    scan->ranges = {0.4};
    auto endpoint = transformPoint(param.base_from_laser, {0.4, 0});
    Position3D hit{endpoint.x, endpoint.y, 0};
    perception::PerceptionInput input;
    input.sensor = scan;
    double stamp = 1;
    auto update = [&]()
    {
        scan->stamp = input.state.stamp = input.now = stamp;
        stamp += 0.1;
        return interface.update(input);
    };

    require(!update().success, "missing robot model must reject the input");
    input.robot = large_robot;
    auto result = update();
    require(result.success && result.map && result.map->cost(hit) == perception::Free,
            "a return inside the input robot footprint must be filtered");

    input.robot = small_robot;
    result = update();
    require(result.success && result.map && result.map->cost(hit) == perception::Lethal,
            "the next frame must use its new robot model and retain an external return");

    scan->ranges = {scan->range_max};
    result = update();
    require(result.success && result.map && result.map->cost(hit) == perception::Free,
            "a no-return ray must still clear the previous obstacle");
    scan->ranges = {0.4};
    input.robot = large_robot;
    result = update();
    require(result.success && result.map && result.map->cost(hit) == perception::Free,
            "the processor must not retain the previous frame's robot model");

    interface.reset();
    input.robot.reset();
    require(!update().success, "reset must not supply a cached robot model");
    input.robot = small_robot;
    result = update();
    require(result.success && result.map && result.map->cost(hit) == perception::Lethal,
            "valid input must recover after a missing robot model");
    std::cout << "perception input robot checks passed\n";
}
