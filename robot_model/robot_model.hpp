#pragma once

namespace robot_model
{
class RobotModel
{
public:
    virtual ~RobotModel() = default;
    virtual const char *type() const = 0;
    virtual double radius() const = 0;
};

class CircleModel final : public RobotModel
{
public:
    /// 半径必须有限且大于零，单位米。
    explicit CircleModel(double radius);
    const char *type() const override;
    double radius() const override;

private:
    const double radius_;
};

} // namespace robot_model
