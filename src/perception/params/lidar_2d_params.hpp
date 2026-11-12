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
    bool track_unknown_space = false;

    // 雷达配置

    // 禁行区和衰减范围
    double forbidden_radius = 0.05;
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
            track_unknown_space = v.value("track_unknown_space", track_unknown_space);
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
