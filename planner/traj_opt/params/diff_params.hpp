#pragma once

#include "base_params.hpp"

namespace traj_opt
{
struct DiffWeights
{
    double time = 10;
    double obstacle = 300000;
    double obstacle_soft = 5000;
    double swarm = 10000;
    double smoothness = 1; // (x, y, yaw) jerk 平方积分
    double kinematics = 300000;
    double feasibility = 300000;
    double sqrvariance = 10000;
};

struct DiffCollisionConfig
{
    double clearance = 0.1;
    double soft_clearance = 0.5;
    double soft_scale = 0.05;
};

struct DiffLimits
{
    double velocity = 1.5;
    double acceleration = 6;
    double angular_velocity = 1.5;
    double angular_acceleration = 6;
};

struct DiffSwarmConfig
{
    double clearance = 0; // 自身包络半径与邻机 clearance 之外的额外距离
    double penalty_scale = 1.5;
    double validation_scale = 1.25;
};

struct DiffValidation
{
    double velocity_tolerance = 0.05;
    double acceleration_tolerance = 0.1;
    double angular_velocity_tolerance = 0.05;
    double angular_acceleration_tolerance = 0.1;
};

class DiffParams : public OptimizerParams
{
public:
    DiffWeights weights;
    DiffCollisionConfig collision;
    DiffLimits limits;
    DiffSwarmConfig swarm;
    DiffValidation validation;

    DiffParams()
    {
        integration_intervals = 16;
        solver.max_iterations = 400;
        solver.g_epsilon = 1e-4;
        solver.delta = 1e-6;
    }

    explicit DiffParams(const std::string &path) : DiffParams()
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
                                       weights.kinematics,
                                       weights.feasibility,
                                       weights.sqrvariance,
                                       collision.clearance,
                                       collision.soft_clearance,
                                       swarm.clearance,
                                       validation.velocity_tolerance,
                                       validation.acceleration_tolerance,
                                       validation.angular_velocity_tolerance,
                                       validation.angular_acceleration_tolerance});
        param_validation::positive({collision.soft_scale, limits.velocity,
                                    limits.acceleration,
                                    limits.angular_velocity,
                                    limits.angular_acceleration,
                                    swarm.penalty_scale,
                                    swarm.validation_scale});
        if (collision.soft_clearance < collision.clearance)
            throw std::invalid_argument("soft_clearance must be >= clearance");
    }

private:
    void readOwn(const nlohmann::json &json)
    {
        if (json.contains("weights"))
        {
            const auto &v = json.at("weights");
            weights.time = v.value("time", weights.time);
            weights.obstacle = v.value("obstacle", weights.obstacle);
            weights.obstacle_soft = v.value("obstacle_soft", weights.obstacle_soft);
            weights.swarm = v.value("swarm", weights.swarm);
            weights.smoothness = v.value("smoothness", weights.smoothness);
            weights.kinematics = v.value("kinematics", weights.kinematics);
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
            limits.angular_velocity = v.value("angular_velocity", limits.angular_velocity);
            limits.angular_acceleration = v.value("angular_acceleration", limits.angular_acceleration);
        }
        if (json.contains("swarm"))
        {
            const auto &v = json.at("swarm");
            swarm.clearance = v.value("clearance", swarm.clearance);
            swarm.penalty_scale = v.value("penalty_scale", swarm.penalty_scale);
            swarm.validation_scale = v.value("validation_scale", swarm.validation_scale);
        }
        if (json.contains("validation"))
        {
            const auto &v = json.at("validation");
            validation.velocity_tolerance = v.value("velocity_tolerance", validation.velocity_tolerance);
            validation.acceleration_tolerance = v.value("acceleration_tolerance", validation.acceleration_tolerance);
            validation.angular_velocity_tolerance = v.value("angular_velocity_tolerance", validation.angular_velocity_tolerance);
            validation.angular_acceleration_tolerance = v.value("angular_acceleration_tolerance", validation.angular_acceleration_tolerance);
        }
    }
};
} // namespace traj_opt
