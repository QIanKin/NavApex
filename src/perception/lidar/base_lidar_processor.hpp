#pragma once
#include <perception/perception_io.hpp>
#include <perception/params/base_params.hpp>

namespace navapex::perception
{
class LidarProcessor
{
public:
    virtual ~LidarProcessor() = default;

    virtual PerceptionResult update(const PerceptionInput &input) = 0;
    
    virtual const PerceptionParams &param() const = 0;
    
    virtual void reset() = 0;
};
} // namespace navapex::perception
