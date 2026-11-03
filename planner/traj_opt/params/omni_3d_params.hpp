#pragma once

#include "base_params.hpp"

namespace traj_opt
{
struct Omni3DWeights
{
    double time = 10;
    double obstacle = 10000;
    double obstacle_soft = 5000;
    double swarm = 10000;
    double smoothness = 1;
    double feasibility = 10000;
    double sqrvariance = 10000;
};

struct CollisionConfig
{
    double clearance = 0.1;
    double soft_clearance = 0.5;
    double soft_scale = 0.05;
};

struct Omni3DLimits
{
    double velocity = 1.5;
    double acceleration = 6;
    double jerk = 20;
};

struct SwarmConfig
{
    double clearance = 0.15;
    double vertical_scale = 2;
    double horizontal_scale = 1;
    double penalty_scale = 1.5;
    double validation_scale = 1.25;
};

struct Omni3DValidation
{
    double velocity_tolerance = 1;
    double acceleration_tolerance = 1;
};

class Omni3DParams : public OptimizerParams
{
public:
    Omni3DWeights weights;
    CollisionConfig collision;
    Omni3DLimits limits;
    SwarmConfig swarm;
    Omni3DValidation validation;
    int max_rebounds = 20;  // 整次规划允许的约束更新重启次数
    int search_cells = 100; // 局部搜索立方窗口每轴格数
    double search_timeout_ms = 200;

    Omni3DParams() = default;

    explicit Omni3DParams(const std::string &path)
    {
        const auto json = readJson(path);
        readCommon(json);
        readOwn(json);
        validate();
    }

    void validate() const override
    {
        OptimizerParams::validate();
        param_validation::nonnegative({weights.time,
                                       weights.obstacle,
                                       weights.obstacle_soft,
                                       weights.swarm,
                                       weights.smoothness,
                                       weights.feasibility,
                                       weights.sqrvariance,
                                       collision.clearance,
                                       collision.soft_clearance,
                                       swarm.clearance,
                                       validation.velocity_tolerance,
                                       validation.acceleration_tolerance});
        param_validation::positive({collision.soft_scale, search_timeout_ms, limits.velocity,
                                    limits.acceleration,
                                    limits.jerk,
                                    swarm.vertical_scale,
                                    swarm.horizontal_scale,
                                    swarm.penalty_scale,
                                    swarm.validation_scale});
        if (collision.soft_clearance < collision.clearance)
            throw std::invalid_argument("soft_clearance must be >= clearance");
        if (max_rebounds < 0 || search_cells < 5 || search_cells > 200)
            throw std::invalid_argument("Invalid MOCHA rebound count or search window");
    }

private:
    void readOwn(const nlohmann::json &json)
    {
        max_rebounds = json.value("max_rebounds", max_rebounds);
        search_cells = json.value("search_cells", search_cells);
        search_timeout_ms = json.value("search_timeout_ms", search_timeout_ms);
        if (json.contains("weights"))
        {
            const auto &v = json.at("weights");
            weights.time = v.value("time", weights.time);
            weights.obstacle = v.value("obstacle", weights.obstacle);
            weights.obstacle_soft = v.value("obstacle_soft", weights.obstacle_soft);
            weights.swarm = v.value("swarm", weights.swarm);
            weights.smoothness = v.value("smoothness", weights.smoothness);
            weights.feasibility = v.value("feasibility", weights.feasibility);
            weights.sqrvariance = v.value("sqrvariance", weights.sqrvariance);
        }
        if (json.contains("collision"))
        {
            const auto &v = json.at("collision");
            collision.clearance = v.value("clearance", collision.clearance);
            collision.soft_clearance = v.value("soft_clearance", collision.soft_clearance);
            collision.soft_scale = v.value("soft_scale", collision.soft_scale);
        }
        if (json.contains("limits"))
        {
            const auto &v = json.at("limits");
            limits.velocity = v.value("velocity", limits.velocity);
            limits.acceleration = v.value("acceleration", limits.acceleration);
            limits.jerk = v.value("jerk", limits.jerk);
        }
        if (json.contains("swarm"))
        {
            const auto &v = json.at("swarm");
            swarm.clearance = v.value("clearance", swarm.clearance);
            swarm.vertical_scale = v.value("vertical_scale", swarm.vertical_scale);
            swarm.horizontal_scale = v.value("horizontal_scale", swarm.horizontal_scale);
            swarm.penalty_scale = v.value("penalty_scale", swarm.penalty_scale);
            swarm.validation_scale = v.value("validation_scale", swarm.validation_scale);
        }
        if (json.contains("validation"))
        {
            const auto &v = json.at("validation");
            validation.velocity_tolerance = v.value("velocity_tolerance", validation.velocity_tolerance);
            validation.acceleration_tolerance = v.value("acceleration_tolerance", validation.acceleration_tolerance);
        }
    }
};
} // namespace traj_opt
