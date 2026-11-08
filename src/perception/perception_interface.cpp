#include "perception_interface.hpp"
#include <perception/lidar/lidar2d/lidar_2d_processor.hpp>
#include <filesystem>

namespace navapex::perception
{
PerceptionInterface::PerceptionInterface(const std::string &type)
{
    auto path = std::filesystem::path(__FILE__).parent_path() / "config" / (type + ".json");
    if(type == "lidar2d"){
        processor_ = std::make_unique<Lidar2DProcessor>(Lidar2DParams(path.string()));
    }
}

PerceptionResult PerceptionInterface::update(const PerceptionInput &input)
{ 
    return processor_->update(input); 
}

const PerceptionParams &PerceptionInterface::param() const
{ 
    return processor_->param(); 
}

void PerceptionInterface::reset()
{ 
    processor_->reset(); 
}

} // namespace navapex::perception
