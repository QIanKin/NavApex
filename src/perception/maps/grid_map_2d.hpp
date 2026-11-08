#pragma once

#include <perception/maps/base_map.hpp>
#include <cmath>
#include <utility>
#include <vector>
#include <tools/log.hpp>

namespace navapex::perception
{

static constexpr char PERCEPTION_TAG[] = "perception";

struct GridGeometry2D
{
    double resolution = 0.05;
    int width = 160, height = 160;
    double origin_x = 0, origin_y = 0;

    bool worldToCell(double px, double py, int &x, int &y) const
    {
        double gx = std::floor(px / resolution) - std::round(origin_x / resolution);
        double gy = std::floor(py / resolution) - std::round(origin_y / resolution);
        if (!std::isfinite(gx) || !std::isfinite(gy) || gx < 0 || gy < 0 || gx >= width || gy >= height)
            return false;
        x = static_cast<int>(gx);
        y = static_cast<int>(gy);
        return true;
    }
};

class GridMap2D final : public OccupancyMap
{
public:
    GridMap2D(GridGeometry2D geometry, std::vector<std::uint8_t> costs)
              : geometry_(geometry), costs_(std::move(costs))
    {
        if (costs_.size() != static_cast<std::size_t>(geometry_.width) * geometry_.height)
        {
            LOGE(PERCEPTION_TAG) << "Grid data size must match geometry";
        }       
    }

    double resolution() const override
    { 
        return geometry_.resolution; 
    }

    Space space() const override 
    {
         return Space::Planar; 
    }

    bool occupied(const Position3D &position) const override
    { 
        return cost(position) >= Forbidden; 
    }

    std::uint8_t cost(const Position3D &position) const override
    {
        int x, y;
        if (!geometry_.worldToCell(position.x, position.y, x, y)) 
            return Unknown;
        auto index = static_cast<std::size_t>(y) * geometry_.width + x;
        return index < costs_.size() ? costs_[index] : Unknown;
    }

    void bounds(Eigen::Vector3d &lower, Eigen::Vector3d &upper) const override
    {
        lower << geometry_.origin_x, geometry_.origin_y, 0;
        upper << geometry_.origin_x + geometry_.width * geometry_.resolution,
                 geometry_.origin_y + geometry_.height * geometry_.resolution, 0;
    }

private:
    GridGeometry2D geometry_;
    std::vector<std::uint8_t> costs_;
};
} // namespace navapex::perception
