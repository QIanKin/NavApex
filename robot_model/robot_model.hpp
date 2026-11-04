#pragma once

#include <Eigen/Core>
#include <map/esdf_map_2d.hpp>

namespace robot_model
{
struct Clearance
{
    double distance = 0;
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
};

class RobotModel
{
public:
    virtual ~RobotModel() = default;
    virtual const char *type() const = 0;
    virtual double radius() const = 0;
    virtual bool clearance(const Eigen::Vector3d &pose, const map::EsdfMap2D &esdf,
                           Clearance &result) const = 0;
};

class CircleModel final : public RobotModel
{
public:
    explicit CircleModel(double radius);
    const char *type() const override;
    double radius() const override;
    bool clearance(const Eigen::Vector3d &pose, const map::EsdfMap2D &esdf,
                   Clearance &result) const override;

private:
    const double radius_;
};

} // namespace robot_model
