#pragma once

#include <Eigen/Core>
#include <robot_model/robot_state.hpp>
#include <memory>
#include <string>

namespace navapex::map
{
class EsdfMap2D;
}

namespace navapex::robot_model
{
class RobotModel
{
public:
    struct Config
    {
        std::string shape = "circle";
        double radius = 0.18;
        double length = 0.36;
        double width = 0.30;
    };

    static std::shared_ptr<const RobotModel> create(const Config &config);
    virtual ~RobotModel() = default;
    virtual const char *type() const = 0;
    virtual double radius() const = 0; // 平面外接圆半径，供规划的圆形包络检查使用。
    virtual bool contains(const Position2D &point, double padding = 0) const = 0; // base 坐标系中的点。
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
};

class CircleModel final : public RobotModel
{
public:
    explicit CircleModel(double radius);
    const char *type() const override;
    double radius() const override;
    bool contains(const Position2D &point, double padding = 0) const override;

private:
    const double radius_;
};

class RectangleModel final : public RobotModel
{
public:
    RectangleModel(double length, double width);
    const char *type() const override;
    double radius() const override;
    bool contains(const Position2D &point, double padding = 0) const override;

private:
    double length_, width_;
};

} // namespace navapex::robot_model
