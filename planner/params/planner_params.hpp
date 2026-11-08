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

    // 控制与跟踪
    double control_period = 0.1;
    double max_velocity = 0.22;
    double max_angular_velocity = 1.0;
    double max_acceleration = 0.4;
    double max_angular_acceleration = 1.5;
    double max_rotation_acceleration = 2.0; // 原地旋转提速；起步旋转同时限制减速。
    double position_gain = 1.5;
    double lateral_gain = 2;
    double heading_gain = 0.8;
    double tracking_distance = 0.25;
    double stopped_velocity = 0.02;
    double stopped_angular_velocity = 0.05;
    double stopped_duration = 0.2;

    explicit PlannerParams(const std::string &path = {})
    {
        if (!path.empty())
        {
            std::ifstream stream(path);
            if (!stream) throw std::invalid_argument("Cannot open config: " + path);
            stream >> json_;
        }
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
            max_velocity = v.value("max_velocity", max_velocity);
            max_angular_velocity = v.value("max_angular_velocity", max_angular_velocity);
            max_acceleration = v.value("max_acceleration", max_acceleration);
            max_angular_acceleration = v.value("max_angular_acceleration", max_angular_acceleration);
            max_rotation_acceleration = v.value("max_rotation_acceleration", max_rotation_acceleration);
            position_gain = v.value("position_gain", position_gain);
            lateral_gain = v.value("lateral_gain", lateral_gain);
            heading_gain = v.value("heading_gain", heading_gain);
            tracking_distance = v.value("tracking_distance", tracking_distance);
            stopped_velocity = v.value("stopped_velocity", stopped_velocity);
            stopped_angular_velocity = v.value("stopped_angular_velocity", stopped_angular_velocity);
            stopped_duration = v.value("stopped_duration", stopped_duration);
        }

    }

private:
    nlohmann::json json_ = nlohmann::json::object();
};
} // namespace navapex::planner
