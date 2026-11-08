#pragma once

#include <perception/perception_io.hpp>
#include <perception/lidar/base_lidar_processor.hpp>
#include <string>

namespace navapex::perception
{


class PerceptionInterface
{
public:
    explicit PerceptionInterface(const std::string &type);
    
    PerceptionResult update(const PerceptionInput &input);
    
    const PerceptionParams &param() const;
    
    void reset();

private:
    std::unique_ptr<LidarProcessor> processor_;
};
} // namespace navapex::perception
