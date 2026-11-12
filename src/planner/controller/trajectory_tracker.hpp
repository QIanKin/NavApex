#pragma once

#include <planner/plan_manage/params/planner_params.hpp>
#include <planner/plan_manage/planner_io.hpp>
#include <robot_model/robot_model.hpp>

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
    explicit TrajectoryTracker(const PlannerParams &param,
                               std::shared_ptr<const robot_model::RobotModel> robot);

    VelocityCommand track(const LocalPlanResult &plan, const RobotState &state,
                          const Pose2D &goal) const;

private:
    PlannerParams param_;
    int integration_intervals_;
    double position_gain_ = 1.5;
    double lateral_gain_ = 2.0;
    double tracking_distance_ = 0.25;
    double robot_max_velocity_ = 0.22;
};
} // namespace navapex::planner
