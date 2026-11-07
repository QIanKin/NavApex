#pragma once

#include "base_params.hpp"

namespace navapex::traj_opt
{

class DiffParams : public OptimizerParams
{
public:
    // 代价权重
    double weight_kinematics = 10000;

    // 运动限制
    double max_angular_velocity = 1.5;
    double max_angular_acceleration = 6;

    // 轨迹验收容差
    double angular_velocity_tolerance = 0.5;
    double angular_acceleration_tolerance = 0.5;

    explicit DiffParams(const std::string &path = {}) : OptimizerParams(path)
    {
        read();
    }

    void read() override
    {
        readBase();
        if (json_.contains("weights"))
        {
            const auto &v = json_.at("weights");
            weight_kinematics = v.value("kinematics", weight_kinematics);
        }
        if (json_.contains("limits"))
        {
            const auto &v = json_.at("limits");
            max_angular_velocity = v.value("angular_velocity", max_angular_velocity);
            max_angular_acceleration = v.value("angular_acceleration", max_angular_acceleration);
        }
        if (json_.contains("validation"))
        {
            const auto &v = json_.at("validation");
            angular_velocity_tolerance = v.value("angular_velocity_tolerance", angular_velocity_tolerance);
            angular_acceleration_tolerance = v.value("angular_acceleration_tolerance", angular_acceleration_tolerance);
        }
    }
};
} // namespace navapex::traj_opt
