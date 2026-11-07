#include "robot_model.hpp"

#include <perception/esdf/esdf_map_2d.hpp>
#include <cmath>
#include <stdexcept>

namespace navapex::robot_model
{
std::shared_ptr<const RobotModel> RobotModel::create(const Config &config)
{
    if (config.shape == "circle") return std::make_shared<CircleModel>(config.radius);
    if (config.shape == "rectangle") return std::make_shared<RectangleModel>(config.length, config.width);
    throw std::invalid_argument("Unknown robot model type: " + config.shape);
}

CircleModel::CircleModel(double radius) : radius_(radius) {}

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

RectangleModel::RectangleModel(double length, double width) : length_(length), width_(width) {}

const char *RectangleModel::type() const { return "rectangle"; }

double RectangleModel::radius() const { return std::hypot(length_, width_) / 2; }

bool RectangleModel::contains(const Position2D &point, double padding) const
{
    return std::abs(point.x) <= length_ / 2 + padding && std::abs(point.y) <= width_ / 2 + padding;
}

void RobotModel::clearance(const Pose2D &pose, const map::EsdfMap2D &esdf,
                           double &distance) const
{
    esdf.query(toPosition2D(pose), distance);
    distance -= radius();
}

void RobotModel::clearance(const Pose2D &pose, const map::EsdfMap2D &esdf,
                           double &distance, Eigen::Vector3d &gradient) const
{
    Eigen::Vector2d xy_gradient;
    esdf.query(toPosition2D(pose), distance, xy_gradient);
    distance -= radius();
    gradient << xy_gradient, 0;
}

void RobotModel::clearanceSmooth(const Pose2D &pose, const map::EsdfMap2D &esdf,
                                 double &distance) const
{
    esdf.querySmooth(toPosition2D(pose), distance);
    distance -= radius();
}

void RobotModel::clearanceSmooth(const Pose2D &pose, const map::EsdfMap2D &esdf,
                                 double &distance, Eigen::Vector3d &gradient) const
{
    Eigen::Vector2d xy_gradient;
    esdf.querySmooth(toPosition2D(pose), distance, xy_gradient);
    distance -= radius();
    gradient << xy_gradient, 0;
}

} // namespace navapex::robot_model
