#pragma once

#include <perception/lidar/base_lidar_processor.hpp>
#include <perception/params/lidar_2d_params.hpp>
#include <perception/maps/grid_map_2d.hpp>

namespace navapex::perception
{

class Lidar2DProcessor final : public LidarProcessor
{

public:
    explicit Lidar2DProcessor(Lidar2DParams param);
    
    PerceptionResult update(const PerceptionInput &input) override;
    
    const Lidar2DParams &param() const override;
    
    void reset() override;

private:
    bool validateTiming(const LaserScan2D &scan, const RobotState &state, double now) const;

    GridGeometry2D geometry() const;
    
    bool cell(Position2D point, int &x, int &y) const;
    
    void moveWindow(Position2D position);
    
    void clearRay(Position2D start, Position2D end);
    
    std::vector<std::uint8_t> inflatedCosts() const;

    Lidar2DParams param_;
    Position2D origin_;
    std::vector<std::uint8_t> raw_;
    
    bool initialized_ = false;
    double last_stamp_ = 0;
};

} // namespace navapex::perception
