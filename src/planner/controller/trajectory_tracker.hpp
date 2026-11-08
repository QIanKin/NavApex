#pragma once

#include <planner/params/planner_params.hpp>
#include <planner/planner_io.hpp>

namespace navapex::planner
{
struct VelocityCommand
{
    double velocity = 0;
    double angular_velocity = 0;
};

class TrajectoryTracker
{
public:
    explicit TrajectoryTracker(const PlannerParams &param);

    VelocityCommand track(const LocalPlanResult &plan, const RobotState &state,
                          const Eigen::Vector3d &goal) const;

private:
    PlannerParams param_;
    int integration_intervals_;
    double max_velocity_ = 0.22;
    double position_gain_ = 1.5;
    double lateral_gain_ = 2.0;
    double tracking_distance_ = 0.25;
};
} // namespace navapex::planner
