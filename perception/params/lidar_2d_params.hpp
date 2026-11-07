#pragma once

#include <perception/params/base_params.hpp>
#include <robot_model/robot_state.hpp>

namespace navapex::perception
{
class Lidar2DParams : public PerceptionParams
{
public:
    // 栅格配置
    double resolution = 0.05;
    int width = 160;
    int height = 160;

    // 雷达配置
    Pose2D base_from_laser{0.10, 0.02, 0.05};
    bool inf_is_free = false;
    double self_filter_padding = 0.02;

    // 禁行区和衰减范围
    double forbidden_radius = 0.20;
    double inflation_radius = 0.65;
    double cost_scaling = 5;

    explicit Lidar2DParams(const std::string &path = {}) : PerceptionParams(path) { read(); }

    void read() override
    {
        const auto &json = json_;
        if (json.contains("timing"))
        {
            const auto &v = json.at("timing");
            max_pose_scan_dt = v.value("max_pose_scan_dt", max_pose_scan_dt);
            max_input_age = v.value("max_input_age", max_input_age);
        }

        if (json.contains("grid"))
        {
            const auto &v = json.at("grid");
            resolution = v.value("resolution", resolution);
            width = v.value("width", width);
            height = v.value("height", height);
        }

        if (json.contains("sensor"))
        {
            const auto &v = json.at("sensor");
            inf_is_free = v.value("inf_is_free", inf_is_free);
            self_filter_padding = v.value("self_filter_padding", self_filter_padding);
            if (v.contains("base_from_laser"))
            {
                const auto &p = v.at("base_from_laser");
                base_from_laser = {p.value("x", base_from_laser.x), p.value("y", base_from_laser.y),
                                   p.value("yaw", base_from_laser.yaw)};
            }
        }

        if (json.contains("inflation"))
        {
            const auto &v = json.at("inflation");
            forbidden_radius = v.value("forbidden_radius", forbidden_radius);
            inflation_radius = v.value("radius", inflation_radius);
            cost_scaling = v.value("cost_scaling", cost_scaling);
        }
    }
};
} // namespace navapex::perception
