#pragma once

#include <perception/maps/base_map.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <memory>
#include <cmath>
#include <stdexcept>

namespace navapex::ros2
{
// 直接持有 Nav2 原始代价图的只读快照，不把 OccupancyGrid 的 0..100 编码混进来。
class Nav2GridMap final : public perception::OccupancyMap
{
public:
    explicit Nav2GridMap(nav2_msgs::msg::Costmap::ConstSharedPtr message) : message_(std::move(message))
    {
        if (!message_) throw std::invalid_argument("Null Nav2 costmap");
        const auto &m = message_->metadata;
        if (m.size_x < 2 || m.size_y < 2 || static_cast<std::uint64_t>(m.size_x) * m.size_y > 4000000 ||
            message_->data.size() != static_cast<std::size_t>(m.size_x) * m.size_y ||
            !std::isfinite(m.resolution) || m.resolution <= 0 || !origin().allFinite() ||
            std::abs(m.origin.orientation.x) > 1e-6 || std::abs(m.origin.orientation.y) > 1e-6 ||
            std::abs(m.origin.orientation.z) > 1e-6 || std::abs(std::abs(m.origin.orientation.w) - 1) > 1e-6)
            throw std::invalid_argument("Invalid or rotated Nav2 costmap geometry");
    }
    int sizeX() const { return static_cast<int>(message_->metadata.size_x); }
    int sizeY() const { return static_cast<int>(message_->metadata.size_y); }
    Eigen::Vector2d origin() const
    { return {message_->metadata.origin.position.x, message_->metadata.origin.position.y}; }
    double resolution() const override { return message_->metadata.resolution; }
    perception::Space space() const override { return perception::Space::Planar; }
    void bounds(Eigen::Vector3d &lower, Eigen::Vector3d &upper) const override
    {
        lower << origin(), 0;
        upper << origin() + Eigen::Vector2d(sizeX(), sizeY()) * resolution(), 0;
    }
    bool occupied(const Position3D &position) const override { return cost(position) >= perception::Forbidden; }
    std::uint8_t cost(const Position3D &position) const override
    {
        if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z))
            return perception::Unknown;
        Eigen::Vector2d local = (Eigen::Vector2d(position.x, position.y) - origin()) / resolution();
        if (local.x() < 0 || local.y() < 0 || local.x() >= sizeX() || local.y() >= sizeY())
            return perception::Unknown;
        int x = static_cast<int>(std::floor(local.x())), y = static_cast<int>(std::floor(local.y()));
        return message_->data[static_cast<std::size_t>(y) * sizeX() + x];
    }
    std::uint8_t cost(int x, int y) const
    {
        if (x < 0 || y < 0 || x >= sizeX() || y >= sizeY()) return 255;
        return message_->data[static_cast<std::size_t>(y) * sizeX() + x];
    }
private:
    nav2_msgs::msg::Costmap::ConstSharedPtr message_;
};
} // namespace navapex::ros2
