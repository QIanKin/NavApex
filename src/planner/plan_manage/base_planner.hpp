#pragma once

#include <planner/plan_manage/params/planner_params.hpp>
#include <planner/plan_manage/planner_io.hpp>

namespace navapex::planner
{
class Planner
{
public:
    virtual ~Planner() = default;

    virtual GlobalPlanResult planGlobal(const GlobalPlanInput &input) = 0;

    virtual LocalPlanResult planLocal(const LocalPlanInput &input) = 0;

    virtual const PlannerParams &param() const = 0;

    virtual void reset() = 0;

    virtual void cancelWarmStart() = 0;
};
} // namespace navapex::planner
