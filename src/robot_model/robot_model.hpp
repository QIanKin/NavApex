#pragma once

#include <Eigen/Core>
#include <robot_model/robot_state.hpp>
#include <memory>
#include <string>
#include <utility>

namespace navapex::map
{
class EsdfMap2D;
}

namespace navapex::robot_model
{
struct RobotLimits
{
    double max_linear_velocity = 0.5;
    double max_linear_acceleration = 1.0;
    double max_angular_velocity = 1.0;
    double max_angular_acceleration = 1.5;
    double max_jerk = 20.0;
};

struct Lidar2DConfig
{
    Pose2D base_from_sensor{{0.10, 0.02}, 0.05};
    bool inf_is_free = false;
    double self_filter_padding = 0.02;
};

struct RobotModelConfig
{
    std::string shape = "circle";
    std::string planner_type = "diff";
    double radius = 0.125;
    double length = 0.25;
    double width = 0.25;
    Lidar2DConfig lidar2d;
    RobotLimits limits;
};

class RobotModel
{
public:
    static std::shared_ptr<const RobotModel> create();
    virtual ~RobotModel() = default;
    virtual const char *type() const = 0;
    const std::string &plannerType() const { return planner_type_; }
    virtual double radius() const = 0; // 平面外接圆半径，供规划的圆形包络检查使用。
    virtual bool contains(const Position2D &point, double padding = 0) const = 0; // base 坐标系中的点。
    virtual const RobotLimits &limits() const = 0;
    virtual const Lidar2DConfig &lidar2D() const = 0;
    // 查询世界坐标系位姿的净距，当前按外接圆计算。
    virtual void clearance(const Pose2D &pose, const map::EsdfMap2D &esdf,
                           double &distance) const;
    // gradient 为净距对 x、y、yaw 的梯度。
    virtual void clearance(const Pose2D &pose, const map::EsdfMap2D &esdf,
                           double &distance, Eigen::Vector3d &gradient) const;
    virtual void clearanceSmooth(const Pose2D &pose, const map::EsdfMap2D &esdf,
                                 double &distance) const;
    virtual void clearanceSmooth(const Pose2D &pose, const map::EsdfMap2D &esdf,
                                 double &distance, Eigen::Vector3d &gradient) const;
protected:
    explicit RobotModel(std::string planner_type = "diff") : planner_type_(std::move(planner_type)) {}

private:
    std::string planner_type_;
};

class CircleModel final : public RobotModel
{
public:
    explicit CircleModel(double radius);
    explicit CircleModel(RobotModelConfig config);
    const char *type() const override;
    double radius() const override;
    bool contains(const Position2D &point, double padding = 0) const override;
    const RobotLimits &limits() const override;
    const Lidar2DConfig &lidar2D() const override;

private:
    const double radius_;
    RobotLimits limits_;
    Lidar2DConfig lidar2d_;
};

class RectangleModel final : public RobotModel
{
public:
    RectangleModel(double length, double width);
    explicit RectangleModel(RobotModelConfig config);
    const char *type() const override;
    double radius() const override;
    bool contains(const Position2D &point, double padding = 0) const override;
    const RobotLimits &limits() const override;
    const Lidar2DConfig &lidar2D() const override;

private:
    double length_, width_;
    RobotLimits limits_;
    Lidar2DConfig lidar2d_;
};

} // namespace navapex::robot_model
