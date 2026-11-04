#pragma once

#include <Eigen/Core>

namespace map
{
struct DistanceQuery2D
{
    double distance = 0;
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
};

class EsdfMap2D
{
public:
    virtual ~EsdfMap2D() = default;

    // 距离单位米，障碍外为正、内部为负；梯度是同一距离函数对世界坐标的偏导。
    // 越界或距离/梯度不可用时返回 false；成功时输出必须有限。
    virtual bool query(const Eigen::Vector2d &position, DistanceQuery2D &result) const = 0;
    virtual double resolution() const = 0;
};
} // namespace map
