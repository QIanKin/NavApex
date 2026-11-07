#pragma once

#include <perception/perception_io.hpp>
#include <perception/lidar/base_lidar_processor.hpp>
#include <robot_model/robot_model.hpp>
#include <string>

namespace navapex::perception
{


class PerceptionInterface
{
public:
    PerceptionInterface(const std::string &type, std::shared_ptr<const robot_model::RobotModel> robot);
    
    PerceptionResult update(const PerceptionInput &input);
    
    const PerceptionParams &param() const;
    
    void reset();

private:
    std::shared_ptr<const robot_model::RobotModel> robot_;
    std::unique_ptr<LidarProcessor> processor_;
};
} // namespace navapex::perception
