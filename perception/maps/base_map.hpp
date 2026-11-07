#pragma once

#include <Eigen/Core>
#include <robot_model/robot_state.hpp>
#include <cstdint>

namespace navapex::perception
{
inline constexpr std::uint8_t Free = 0;
inline constexpr std::uint8_t Forbidden = 253;
inline constexpr std::uint8_t Lethal = 254;
inline constexpr std::uint8_t Unknown = 255;

enum class Space { 
    Planar,   // 2d
    Spatial   // 3d
};

class OccupancyMap
{
public:
    virtual ~OccupancyMap() = default;

    virtual bool occupied(const Position3D &position) const = 0;
    
    virtual std::uint8_t cost(const Position3D &position) const = 0;
    
    virtual double resolution() const = 0;
    
    virtual Space space() const = 0;
    
    virtual void bounds(Eigen::Vector3d &lower, Eigen::Vector3d &upper) const = 0;
};
} // namespace navapex::perception
