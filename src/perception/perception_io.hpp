#pragma once

#include <perception/maps/base_map.hpp>
#include <robot_model/robot_state.hpp>
#include <limits>
#include <memory>
#include <variant>
#include <vector>

namespace navapex::perception
{
struct LaserScan2D
{
    double stamp = std::numeric_limits<double>::quiet_NaN();
    double angle_min = 0;
    double angle_increment = 0;
    double range_min = 0.05;
    double range_max = 10;
    std::vector<double> ranges;
    double time_increment = 0;
};

struct TimedPoint3D
{
    double x = 0, y = 0, z = 0;
    double time_offset = 0;
};

struct PointCloud3D
{
    double stamp = std::numeric_limits<double>::quiet_NaN();
    std::vector<TimedPoint3D> points;
};

using SensorFrame = std::variant<std::shared_ptr<const LaserScan2D>, std::shared_ptr<const PointCloud3D>>;

struct PerceptionInput
{
    RobotState state;
    SensorFrame sensor;
    double now = std::numeric_limits<double>::quiet_NaN();
};

struct PerceptionResult
{
    bool success = false;
    double stamp = std::numeric_limits<double>::quiet_NaN();
    std::shared_ptr<const OccupancyMap> map;
};
} // namespace navapex::perception
