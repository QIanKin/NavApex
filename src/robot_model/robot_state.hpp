#pragma once

#include <cmath>
#include <limits>

namespace navapex
{

struct Quaternion
{
    double w = 1, x = 0, y = 0, z = 0;
};

struct Position2D
{
    double x = 0, y = 0;
};

struct Position3D
{
    double x = 0, y = 0, z = 0;
};

struct Pose2D
{
    double x = 0, y = 0, yaw = 0;
};

struct Pose3D
{
    Position3D position;
    Quaternion orientation;
};

struct Velocity3D
{
    double x = 0, y = 0, z = 0;
};

struct Twist3D
{
    Velocity3D linear;
    Velocity3D angular;
};

struct Acceleration3D
{
    double x = 0, y = 0, z = 0;
};

struct Accel3D
{
    Acceleration3D linear;
    Acceleration3D angular;
};

struct RobotState
{
    Pose3D pose;
    Twist3D twist;
    Accel3D acceleration;
    double stamp = std::numeric_limits<double>::quiet_NaN();
};

inline Position2D toPosition2D(const Pose2D &pose)
{
    return {pose.x, pose.y};
}

inline Position2D toPosition2D(const Pose3D &pose)
{
    return {pose.position.x, pose.position.y};
}

inline Pose2D toPose2D(const Pose3D &pose)
{
    const auto &p = pose.position;
    const auto &q = pose.orientation;
    double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    return {p.x, p.y, yaw};
}

inline Position2D transformPoint(const Pose2D &target_from_source, Position2D point)
{
    double c = std::cos(target_from_source.yaw), s = std::sin(target_from_source.yaw);
    return {target_from_source.x + c * point.x - s * point.y,
            target_from_source.y + s * point.x + c * point.y};
}

} // namespace navapex
