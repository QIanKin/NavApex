#pragma once

#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <string>

namespace navapex::traj_opt
{
class OptimizerParams
{
public:
    // L-BFGS 求解器
    int solver_mem_size = 16;
    int solver_max_iterations = 200;
    int solver_past = 3;
    int solver_max_linesearch = 40;
    double solver_g_epsilon = 1e-5;
    double solver_delta = 1e-2;
    double solver_min_step = 1e-32;
    double solver_max_step = 1e20;
    double solver_f_dec_coeff = 1e-4;
    double solver_s_curv_coeff = 0.9;
    double solver_xtol = 1e-16;
    int solver_abs_curv_cond = 1;

    // 基础代价权重
    double weight_time = 10;
    double weight_obstacle = 10000;
    double weight_obstacle_soft = 5000;
    double weight_swarm = 10000;
    double weight_smoothness = 1;
    double weight_feasibility = 10000;
    double weight_sqrvariance = 10000;

    // 障碍净距
    double collision_clearance = 0.1;
    double collision_soft_clearance = 0.5;
    double collision_soft_scale = 0.05;

    // 运动限制
    double max_velocity = 1.5;
    double max_acceleration = 6;

    // 多机器人避障
    double swarm_clearance = 0.15;
    double swarm_penalty_scale = 1.5;
    double swarm_validation_scale = 1.25;

    // 轨迹验收容差
    double velocity_tolerance = 1;
    double acceleration_tolerance = 1;

    // 迭代与搜索
    int integration_intervals = 5;
    int max_attempts = 3;

    explicit OptimizerParams(const std::string &path = {})
    {
        if (path.empty())
            return;
        std::ifstream stream(path);
        if (!stream)
            throw std::invalid_argument("Cannot open config: " + path);
        stream >> json_;
    }
    virtual ~OptimizerParams() = default;
    virtual void read() = 0;

protected:
    void readBase()
    {
        integration_intervals = json_.value("integration_intervals", integration_intervals);
        max_attempts = json_.value("max_attempts", max_attempts);
        if (json_.contains("solver"))
        {
            const auto &v = json_.at("solver");
            solver_mem_size = v.value("mem_size", solver_mem_size);
            solver_max_iterations = v.value("max_iterations", solver_max_iterations);
            solver_past = v.value("past", solver_past);
            solver_max_linesearch = v.value("max_linesearch", solver_max_linesearch);
            solver_g_epsilon = v.value("g_epsilon", solver_g_epsilon);
            solver_delta = v.value("delta", solver_delta);
            solver_min_step = v.value("min_step", solver_min_step);
            solver_max_step = v.value("max_step", solver_max_step);
            solver_f_dec_coeff = v.value("f_dec_coeff", solver_f_dec_coeff);
            solver_s_curv_coeff = v.value("s_curv_coeff", solver_s_curv_coeff);
            solver_xtol = v.value("xtol", solver_xtol);
            solver_abs_curv_cond = v.value("abs_curv_cond", solver_abs_curv_cond);
        }
        if (json_.contains("weights"))
        {
            const auto &v = json_.at("weights");
            weight_time = v.value("time", weight_time);
            weight_obstacle = v.value("obstacle", weight_obstacle);
            weight_obstacle_soft = v.value("obstacle_soft", weight_obstacle_soft);
            weight_swarm = v.value("swarm", weight_swarm);
            weight_smoothness = v.value("smoothness", weight_smoothness);
            weight_feasibility = v.value("feasibility", weight_feasibility);
            weight_sqrvariance = v.value("sqrvariance", weight_sqrvariance);
        }
        if (json_.contains("collision"))
        {
            const auto &v = json_.at("collision");
            collision_clearance = v.value("clearance", collision_clearance);
            collision_soft_clearance = v.value("soft_clearance", collision_soft_clearance);
            collision_soft_scale = v.value("soft_scale", collision_soft_scale);
        }
        if (json_.contains("limits"))
        {
            const auto &v = json_.at("limits");
            max_velocity = v.value("velocity", max_velocity);
            max_acceleration = v.value("acceleration", max_acceleration);
        }
        if (json_.contains("swarm"))
        {
            const auto &v = json_.at("swarm");
            swarm_clearance = v.value("clearance", swarm_clearance);
            swarm_penalty_scale = v.value("penalty_scale", swarm_penalty_scale);
            swarm_validation_scale = v.value("validation_scale", swarm_validation_scale);
        }
        if (json_.contains("validation"))
        {
            const auto &v = json_.at("validation");
            velocity_tolerance = v.value("velocity_tolerance", velocity_tolerance);
            acceleration_tolerance = v.value("acceleration_tolerance", acceleration_tolerance);
        }
    }

    nlohmann::json json_ = nlohmann::json::object();
};
} // namespace navapex::traj_opt
