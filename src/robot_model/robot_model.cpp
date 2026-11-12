#include "robot_model.hpp"

#include <perception/esdf/esdf_map_2d.hpp>
#include <nlohmann/json.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace navapex::robot_model
{
std::shared_ptr<const RobotModel> RobotModel::create()
{
    auto path = std::filesystem::path(__FILE__).parent_path() / "config" / "robot.json";
    std::ifstream stream(path);
    if (!stream) throw std::invalid_argument("Cannot open config: " + path.string());
    nlohmann::json config;
    stream >> config;
    RobotModelConfig model;
    model.shape = config.value("shape", model.shape);
    model.planner_type = config.value("planner_type", model.planner_type);
    model.radius = config.value("radius", model.radius);
    model.length = config.value("length", model.length);
    model.width = config.value("width", model.width);
    if (config.contains("sensors") && config.at("sensors").contains("lidar2d"))
    {
        const auto &sensor = config.at("sensors").at("lidar2d");
        model.lidar2d.base_from_sensor = {{sensor.value("x", model.lidar2d.base_from_sensor.position.x),
                                           sensor.value("y", model.lidar2d.base_from_sensor.position.y)},
                                          sensor.value("yaw", model.lidar2d.base_from_sensor.yaw)};
        model.lidar2d.inf_is_free = sensor.value("inf_is_free", model.lidar2d.inf_is_free);
        model.lidar2d.self_filter_padding = sensor.value("self_filter_padding", model.lidar2d.self_filter_padding);
    }
    if (config.contains("limits"))
    {
        const auto &limits = config.at("limits");
        model.limits.max_linear_velocity = limits.value("max_linear_velocity", model.limits.max_linear_velocity);
        model.limits.max_linear_acceleration = limits.value("max_linear_acceleration", model.limits.max_linear_acceleration);
        model.limits.max_angular_velocity = limits.value("max_angular_velocity", model.limits.max_angular_velocity);
        model.limits.max_angular_acceleration = limits.value("max_angular_acceleration", model.limits.max_angular_acceleration);
        model.limits.max_jerk = limits.value("max_jerk", model.limits.max_jerk);
    }
    const auto &limits = model.limits;
    if (!std::isfinite(model.lidar2d.base_from_sensor.position.x) ||
        !std::isfinite(model.lidar2d.base_from_sensor.position.y) ||
        !std::isfinite(model.lidar2d.base_from_sensor.yaw) ||
        !std::isfinite(model.lidar2d.self_filter_padding) || model.lidar2d.self_filter_padding < 0 ||
        !std::isfinite(limits.max_linear_velocity) || limits.max_linear_velocity <= 0 ||
        !std::isfinite(limits.max_linear_acceleration) || limits.max_linear_acceleration <= 0 ||
        !std::isfinite(limits.max_angular_velocity) || limits.max_angular_velocity <= 0 ||
        !std::isfinite(limits.max_angular_acceleration) || limits.max_angular_acceleration <= 0 ||
        !std::isfinite(limits.max_jerk) || limits.max_jerk <= 0)
        throw std::invalid_argument("Robot limits and sensor pose must be finite and positive");
    if ((model.shape == "circle" && (!std::isfinite(model.radius) || model.radius <= 0)) ||
        (model.shape == "rectangle" && (!std::isfinite(model.length) || model.length <= 0 ||
                                         !std::isfinite(model.width) || model.width <= 0)))
        throw std::invalid_argument("Robot dimensions must be finite and positive");
    if (model.shape == "circle") return std::make_shared<CircleModel>(std::move(model));
    if (model.shape == "rectangle") return std::make_shared<RectangleModel>(std::move(model));
    throw std::invalid_argument("Unknown robot model type: " + model.shape);
}

CircleModel::CircleModel(double radius) : radius_(radius) {}

CircleModel::CircleModel(RobotModelConfig config)
    : RobotModel(std::move(config.planner_type)), radius_(config.radius), limits_(config.limits), lidar2d_(config.lidar2d) {}

const char *CircleModel::type() const
{
    return "circle";
}

double CircleModel::radius() const
{
    return radius_;
}

bool CircleModel::contains(const Position2D &point, double padding) const
{
    return std::hypot(point.x, point.y) <= radius_ + padding;
}

const RobotLimits &CircleModel::limits() const { return limits_; }

const Lidar2DConfig &CircleModel::lidar2D() const { return lidar2d_; }

RectangleModel::RectangleModel(double length, double width) : length_(length), width_(width) {}

RectangleModel::RectangleModel(RobotModelConfig config)
    : RobotModel(std::move(config.planner_type)), length_(config.length), width_(config.width), limits_(config.limits),
      lidar2d_(config.lidar2d) {}

const char *RectangleModel::type() const { return "rectangle"; }

double RectangleModel::radius() const { return std::hypot(length_, width_) / 2; }

bool RectangleModel::contains(const Position2D &point, double padding) const
{
    return std::abs(point.x) <= length_ / 2 + padding && std::abs(point.y) <= width_ / 2 + padding;
}

const RobotLimits &RectangleModel::limits() const { return limits_; }

const Lidar2DConfig &RectangleModel::lidar2D() const { return lidar2d_; }

void RobotModel::clearance(const Pose2D &pose, const map::EsdfMap2D &esdf,
                           double &distance) const
{
    esdf.query(pose.position, distance);
    distance -= radius();
}

void RobotModel::clearance(const Pose2D &pose, const map::EsdfMap2D &esdf,
                           double &distance, Eigen::Vector3d &gradient) const
{
    Eigen::Vector2d xy_gradient;
    esdf.query(pose.position, distance, xy_gradient);
    distance -= radius();
    gradient << xy_gradient, 0;
}

void RobotModel::clearanceSmooth(const Pose2D &pose, const map::EsdfMap2D &esdf,
                                 double &distance) const
{
    esdf.querySmooth(pose.position, distance);
    distance -= radius();
}

void RobotModel::clearanceSmooth(const Pose2D &pose, const map::EsdfMap2D &esdf,
                                 double &distance, Eigen::Vector3d &gradient) const
{
    Eigen::Vector2d xy_gradient;
    esdf.querySmooth(pose.position, distance, xy_gradient);
    distance -= radius();
    gradient << xy_gradient, 0;
}

} // namespace navapex::robot_model
