#include "esdf_map_2d.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navapex::map
{

EsdfMap2D::EsdfMap2D(const perception::OccupancyMap &map) : resolution_(map.resolution())
{
    Eigen::Vector3d lower, upper;
    map.bounds(lower, upper);
    if (map.space() != perception::Space::Planar ||
        !lower.allFinite() || !upper.allFinite() || !std::isfinite(resolution_) || resolution_ <= 0)
        throw std::invalid_argument("Invalid ESDF grid geometry");
    origin_ = lower.head<2>();
    Eigen::Vector2d cells = ((upper - lower).head<2>() / resolution_).array().round();
    if (!cells.allFinite() || cells.minCoeff() < 2 || cells.maxCoeff() > 4000000 || cells.prod() > 4000000)
        throw std::invalid_argument("Invalid ESDF grid dimensions");
    width_ = static_cast<int>(cells.x());
    height_ = static_cast<int>(cells.y());

    // 外圈和未知按占据处理；253 禁行区不作为实体障碍参与距离场。
    cv::Mat free(height_ + 2, width_ + 2, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < height_; ++y)
        for (int x = 0; x < width_; ++x)
        {
            Position3D position{origin_.x() + (x + 0.5) * resolution_,
                                origin_.y() + (y + 0.5) * resolution_, 0};
            free.at<unsigned char>(y + 1, x + 1) = map.cost(position) < perception::Lethal ? 255 : 0;
        }
    cv::Mat occupied, outside, inside;
    cv::bitwise_not(free, occupied);
    cv::distanceTransform(free, outside, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::distanceTransform(occupied, inside, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    distances_.create(free.size(), CV_32FC1);
    for (int y = 0; y < free.rows; ++y)
        for (int x = 0; x < free.cols; ++x)
        {
            bool empty = free.at<unsigned char>(y, x) != 0;
            double distance = empty ? outside.at<float>(y, x) - 0.5 : 0.5 - inside.at<float>(y, x);
            distances_.at<float>(y, x) = static_cast<float>(distance * resolution_);
        }
}

void EsdfMap2D::query(const Position2D &position, double &distance) const
{
    Eigen::Vector2d grid = (Eigen::Vector2d(position.x, position.y) - origin_) / resolution_;
    Eigen::Vector2d clamped(std::clamp(grid.x(), 0.0, static_cast<double>(width_)),
                           std::clamp(grid.y(), 0.0, static_cast<double>(height_)));
    // 距离定义在格心，使用双线性插值。
    Eigen::Vector2d sample = clamped + Eigen::Vector2d::Constant(0.5);
    int x = static_cast<int>(std::floor(sample.x())), y = static_cast<int>(std::floor(sample.y()));
    double u = sample.x() - x, v = sample.y() - y;
    double a = distances_.at<float>(y, x), b = distances_.at<float>(y, x + 1);
    double c = distances_.at<float>(y + 1, x), d = distances_.at<float>(y + 1, x + 1);
    distance = (1 - v) * ((1 - u) * a + u * b) + v * ((1 - u) * c + u * d);
    // 地图外属于未知占据区域，延伸为负距离。
    Eigen::Vector2d offset = (grid - clamped) * resolution_;
    double outside = std::hypot(offset.x(), offset.y());
    if (outside > 0) distance -= outside;
}

void EsdfMap2D::query(const Position2D &position, double &distance, Eigen::Vector2d &gradient) const
{
    Eigen::Vector2d grid = (Eigen::Vector2d(position.x, position.y) - origin_) / resolution_;
    Eigen::Vector2d clamped(std::clamp(grid.x(), 0.0, static_cast<double>(width_)),
                           std::clamp(grid.y(), 0.0, static_cast<double>(height_)));
    // 梯度直接对同一双线性插值函数求导。
    Eigen::Vector2d sample = clamped + Eigen::Vector2d::Constant(0.5);
    int x = static_cast<int>(std::floor(sample.x())), y = static_cast<int>(std::floor(sample.y()));
    double u = sample.x() - x, v = sample.y() - y;
    double a = distances_.at<float>(y, x), b = distances_.at<float>(y, x + 1);
    double c = distances_.at<float>(y + 1, x), d = distances_.at<float>(y + 1, x + 1);
    distance = (1 - v) * ((1 - u) * a + u * b) + v * ((1 - u) * c + u * d);
    gradient << ((1 - v) * (b - a) + v * (d - c)) / resolution_,
                ((1 - u) * (c - a) + u * (d - b)) / resolution_;
    // 地图外延伸负距离，梯度指向地图内。
    Eigen::Vector2d offset = (grid - clamped) * resolution_;
    double outside = std::hypot(offset.x(), offset.y());
    if (outside > 0)
    {
        distance -= outside;
        if (grid.x() != clamped.x()) gradient.x() = 0;
        if (grid.y() != clamped.y()) gradient.y() = 0;
        gradient -= offset / outside;
    }
}

void EsdfMap2D::querySmooth(const Position2D &position, double &distance) const
{
    Eigen::Vector2d sample = (Eigen::Vector2d(position.x, position.y) - origin_) / resolution_ + Eigen::Vector2d::Constant(0.5);

    Eigen::Vector2d clamped(std::clamp(sample.x(), -1.0, static_cast<double>(distances_.cols)),
                           std::clamp(sample.y(), -1.0, static_cast<double>(distances_.rows)));
    int x = static_cast<int>(std::floor(clamped.x())), y = static_cast<int>(std::floor(clamped.y()));
    double fraction[2] = {clamped.x() - x, clamped.y() - y};
    double weight[2][4];
    // 只计算 Catmull–Rom 插值权重，不计算导数。
    for (int axis = 0; axis < 2; ++axis)
    {
        double t = fraction[axis], t2 = t * t, t3 = t2 * t;
        weight[axis][0] = 0.5 * (-t3 + 2 * t2 - t);
        weight[axis][1] = 0.5 * (3 * t3 - 5 * t2 + 2);
        weight[axis][2] = 0.5 * (-3 * t3 + 4 * t2 + t);
        weight[axis][3] = 0.5 * (t3 - t2);
    }
    distance = 0;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
        {
            double value = distances_.at<float>(std::clamp(y + row - 1, 0, distances_.rows - 1),
                                                std::clamp(x + col - 1, 0, distances_.cols - 1));
            distance += weight[0][col] * weight[1][row] * value;
        }

    Eigen::Vector2d offset = (sample - clamped) * resolution_;
    double outside = std::hypot(offset.x(), offset.y());
    if (outside > 0) distance -= std::hypot(outside, resolution_) - resolution_;
}

void EsdfMap2D::querySmooth(const Position2D &position, double &distance, Eigen::Vector2d &gradient) const
{
    Eigen::Vector2d sample = (Eigen::Vector2d(position.x, position.y) - origin_) / resolution_ + Eigen::Vector2d::Constant(0.5);

    // 在占据边框之外重复端点值，再平滑延伸负距离及梯度。
    Eigen::Vector2d clamped(std::clamp(sample.x(), -1.0, static_cast<double>(distances_.cols)),
                           std::clamp(sample.y(), -1.0, static_cast<double>(distances_.rows)));
    int x = static_cast<int>(std::floor(clamped.x())), y = static_cast<int>(std::floor(clamped.y()));
    double fraction[2] = {clamped.x() - x, clamped.y() - y};
    double weight[2][4], derivative[2][4];
    for (int axis = 0; axis < 2; ++axis)
    {
        double t = fraction[axis], t2 = t * t, t3 = t2 * t;
        weight[axis][0] = 0.5 * (-t3 + 2 * t2 - t);
        weight[axis][1] = 0.5 * (3 * t3 - 5 * t2 + 2);
        weight[axis][2] = 0.5 * (-3 * t3 + 4 * t2 + t);
        weight[axis][3] = 0.5 * (t3 - t2);
        derivative[axis][0] = 0.5 * (-3 * t2 + 4 * t - 1);
        derivative[axis][1] = 0.5 * (9 * t2 - 10 * t);
        derivative[axis][2] = 0.5 * (-9 * t2 + 8 * t + 1);
        derivative[axis][3] = 0.5 * (3 * t2 - 2 * t);
    }
    distance = 0;
    gradient.setZero();
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
        {
            double value = distances_.at<float>(std::clamp(y + row - 1, 0, distances_.rows - 1),
                                                std::clamp(x + col - 1, 0, distances_.cols - 1));
            distance += weight[0][col] * weight[1][row] * value;
            gradient.x() += derivative[0][col] * weight[1][row] * value;
            gradient.y() += weight[0][col] * derivative[1][row] * value;
        }
    gradient /= resolution_;

    Eigen::Vector2d offset = (sample - clamped) * resolution_;
    double outside = std::hypot(offset.x(), offset.y());
    if (outside > 0)
    {
        double scale = std::hypot(outside, resolution_);
        distance -= scale - resolution_;
        if (sample.x() != clamped.x()) gradient.x() = 0;
        if (sample.y() != clamped.y()) gradient.y() = 0;
        gradient -= offset / scale;
    }
}

double EsdfMap2D::resolution() const { return resolution_; }
} // namespace navapex::map
