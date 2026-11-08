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
};
} // namespace navapex::planner
