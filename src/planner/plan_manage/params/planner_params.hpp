#pragma once

#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <string>

namespace navapex::planner
{
class PlannerParams
{
public:
    // 局部规划
    double lookahead_distance = 1.2;
    double waypoint_spacing = 0.4;
    double cruise_speed = 0.18;
    double position_tolerance = 0.10;
    double yaw_tolerance = 0.10;
    double rotate_threshold = 1.05;
    double start_yaw_tolerance = 0.35;

    // 旋转控制与运动限制
    double control_period = 0.1;
    double max_rotation_velocity = 1.0;
    double max_acceleration = 0.4;
    double max_rotation_acceleration = 1.5;
    double heading_gain = 0.8;

    PlannerParams() = default;
    explicit PlannerParams(const std::string &path)
    {
        std::ifstream stream(path);
        if (!stream) throw std::invalid_argument("Cannot open config: " + path);
        stream >> json_;
        read();
    }

    void read()
    {
        const auto &json = json_;
        if (json.contains("local"))
        {
            const auto &v = json.at("local");
            lookahead_distance = v.value("lookahead_distance", lookahead_distance);
            waypoint_spacing = v.value("waypoint_spacing", waypoint_spacing);
            cruise_speed = v.value("cruise_speed", cruise_speed);
            position_tolerance = v.value("position_tolerance", position_tolerance);
            yaw_tolerance = v.value("yaw_tolerance", yaw_tolerance);
            rotate_threshold = v.value("rotate_threshold", rotate_threshold);
            start_yaw_tolerance = v.value("start_yaw_tolerance", start_yaw_tolerance);
        }
        if (json.contains("controller"))
        {
            const auto &v = json.at("controller");
            control_period = v.value("period", control_period);
            max_rotation_velocity = v.value("max_rotation_velocity", max_rotation_velocity);
            max_acceleration = v.value("max_acceleration", max_acceleration);
            max_rotation_acceleration = v.value("max_rotation_acceleration", max_rotation_acceleration);
            heading_gain = v.value("heading_gain", heading_gain);
        }

    }

private:
    nlohmann::json json_ = nlohmann::json::object();
};

// Loaded once by the application layer. Component configuration locations are
// fixed by the module layout and are not part of this user-facing object.
struct PlannerConfig
{
    PlannerParams planner;

    static PlannerConfig defaultConfig();
};
} // namespace navapex::planner
