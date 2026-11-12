#pragma once

#include <Eigen/Core>
#include <cmath>
#include <limits>

namespace navapex
{

// position
struct Position2D
{
    double x = 0, y = 0;
    bool operator==(const Position2D &b) const
    { 
        return x == b.x && y == b.y; 
    }
    Eigen::Vector2d toEigen() const
    {
        return {x, y};
    }
    static Position2D fromEigen(const Eigen::Vector2d &value)
    {
        return {value.x(), value.y()};
    }
};

struct Position3D
{
    double x = 0, y = 0, z = 0;
    bool operator==(const Position3D &b) const
    {
        return x == b.x && y == b.y && z == b.z;
    }
    Position2D toPlanar() const
    {
        return {x, y};
    }
    Eigen::Vector3d toEigen() const
    {
        return {x, y, z};
    }
    static Position3D fromEigen(const Eigen::Vector3d &value)
    {
        return {value.x(), value.y(), value.z()};
    }
};

// pose
struct Quaternion
{
    double w = 1, x = 0, y = 0, z = 0;
    bool operator==(const Quaternion &b) const
    {
        return w == b.w && x == b.x && y == b.y && z == b.z;
    }
    Position3D rotate(Position3D vector) const
    {
        double tx = 2 * (y * vector.z - z * vector.y);
        double ty = 2 * (z * vector.x - x * vector.z);
        double tz = 2 * (x * vector.y - y * vector.x);
        return {vector.x + w * tx + (y * tz - z * ty),
                vector.y + w * ty + (z * tx - x * tz),
                vector.z + w * tz + (x * ty - y * tx)};
    }
};

struct Pose2D
{
    Position2D position;
    double yaw = 0;
    bool operator==(const Pose2D &b) const
    { 
        return position == b.position && yaw == b.yaw;
    }
    Eigen::Vector3d toEigen() const
    {
        return {position.x, position.y, yaw};
    }
    static Pose2D fromEigen(const Eigen::Vector3d &value)
    {
        return {{value.x(), value.y()},value.z()};
    }
    Position2D localToParent(Position2D point_in_local) const
    {
        double c = std::cos(yaw);
        double s = std::sin(yaw);
        return {position.x + c * point_in_local.x - s * point_in_local.y,
                position.y + s * point_in_local.x + c * point_in_local.y};
    }
};

struct Pose3D
{
    Position3D position;
    Quaternion orientation;
    bool operator==(const Pose3D &b) const
    {
        return position == b.position && orientation == b.orientation;
    }
    Pose2D toPlanar() const
    {
        auto &q = orientation;
        double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        return {position.toPlanar(), yaw};
    }
    Position3D localToParent(Position3D point_in_local) const
    {
        Position3D rotated = orientation.rotate(point_in_local);
        return {rotated.x + position.x, rotated.y + position.y, rotated.z + position.z};
    }
};

// vel
struct LinearVelocity2D
{
    double x = 0, y = 0;
};

struct AngularVelocity2D
{
    double z = 0;
};

struct WorldTwist2D
{
    LinearVelocity2D linear;
    AngularVelocity2D angular;
    Eigen::Vector3d toEigen() const
    {
        return {linear.x, linear.y, angular.z};
    }
};

struct BodyTwist2D
{
    LinearVelocity2D linear;
    AngularVelocity2D angular;
    WorldTwist2D toWorld(const Pose2D &world_from_body) const
    {
        double c = std::cos(world_from_body.yaw);
        double s = std::sin(world_from_body.yaw);
        return {{c * linear.x - s * linear.y,
                 s * linear.x + c * linear.y},
                {this->angular.z}};
    }
    WorldTwist2D toWorld(const Pose3D &world_from_body) const
    {
        return toWorld(world_from_body.toPlanar());
    }
};

struct LinearVelocity3D
{
    double x = 0, y = 0, z = 0;
    LinearVelocity2D toPlanar() const
    {
        return {x, y};
    }
};

struct AngularVelocity3D
{
    double x = 0, y = 0, z = 0;
    AngularVelocity2D toPlanar() const
    {
        return {z};
    }
};

struct WorldTwist3D
{
    LinearVelocity3D linear;
    AngularVelocity3D angular;
    WorldTwist2D toPlanar() const
    {
        return {linear.toPlanar(), angular.toPlanar()};
    }
};

struct BodyTwist3D
{
    LinearVelocity3D linear;
    AngularVelocity3D angular;
    BodyTwist2D toPlanar() const
    {
        return {linear.toPlanar(), angular.toPlanar()};
    }
    WorldTwist3D toWorld(const Pose3D &world_from_body) const
    {
        auto &rotation = world_from_body.orientation;
        auto world_linear = rotation.rotate({linear.x, linear.y, linear.z});
        auto world_angular = rotation.rotate({angular.x, angular.y, angular.z});
        return {{world_linear.x, world_linear.y, world_linear.z},
                {world_angular.x, world_angular.y, world_angular.z}};
    }
};

// acc
struct LinearAcceleration2D
{
    double x = 0, y = 0;
};

struct AngularAcceleration2D
{
    double z = 0;
};

struct WorldAcceleration2D
{
    LinearAcceleration2D linear;
    AngularAcceleration2D angular;
    Eigen::Vector3d toEigen() const
    {
        return {linear.x, linear.y, angular.z};
    }
};

struct BodyAcceleration2D
{
    LinearAcceleration2D linear;
    AngularAcceleration2D angular;
    WorldAcceleration2D toWorld(const Pose2D &world_from_body) const
    {
        double c = std::cos(world_from_body.yaw);
        double s = std::sin(world_from_body.yaw);
        return {{c * linear.x - s * linear.y,
                 s * linear.x + c * linear.y},
                 angular};
    }
    WorldAcceleration2D toWorld(const Pose3D &world_from_body) const
    {
        return toWorld(world_from_body.toPlanar());
    }
};

struct LinearAcceleration3D
{
    double x = 0, y = 0, z = 0;
    LinearAcceleration2D toPlanar() const
    {
        return {x, y};
    }
};

struct AngularAcceleration3D
{
    double x = 0, y = 0, z = 0;
    AngularAcceleration2D toPlanar() const
    {
        return {z};
    }
};

struct BodyAcceleration3D
{
    LinearAcceleration3D linear;
    AngularAcceleration3D angular;
    BodyAcceleration2D toPlanar() const
    {
        return {linear.toPlanar(), angular.toPlanar()};
    }
};

// 状态定义
struct RobotState
{
    Pose3D pose;
    BodyTwist3D twist;
    BodyAcceleration3D acceleration;
    double stamp = std::numeric_limits<double>::quiet_NaN();
};

}
