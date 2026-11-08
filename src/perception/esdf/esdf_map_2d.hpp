#pragma once

#include <Eigen/Core>
#include <perception/maps/base_map.hpp>
#include <robot_model/robot_state.hpp>
#include <opencv2/core.hpp>

namespace navapex::map
{
class EsdfMap2D
{
public:
    explicit EsdfMap2D(const perception::OccupancyMap &map);

    void query(const Position2D &position, double &distance) const;

    void query(const Position2D &position, double &distance, Eigen::Vector2d &gradient) const;

    void querySmooth(const Position2D &position, double &distance) const;

    void querySmooth(const Position2D &position, double &distance, Eigen::Vector2d &gradient) const;

    double resolution() const;

private:
    Eigen::Vector2d origin_;
    double resolution_;
    int width_, height_;
    cv::Mat distances_;
};
} // namespace navapex::map
