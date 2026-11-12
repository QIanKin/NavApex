#pragma once

#include <optimizer_interface.hpp>
#include <planner/base_planner.hpp>
#include <planner/path_plan/searcher_interface.hpp>

namespace navapex::planner
{
class DiffPlanner final : public Planner
{
public:
    DiffPlanner(std::shared_ptr<const robot_model::RobotModel> robot, PlannerParams param = PlannerParams{});

    GlobalPlanResult planGlobal(const GlobalPlanInput &input) override;

    LocalPlanResult planLocal(const LocalPlanInput &input) override;
    
    const PlannerParams &param() const override;
    
    void reset() override;
    
    void cancelWarmStart() override;

private:
    enum class MotionPhase
    {
        NewPath,
        StartRotating,
        Rotating,
        Navigating,
        GoalRotating,
        DirectionAlign,
        Finished
    };

    double findAimDistance(const Eigen::Vector2d &position) const;
    
    Eigen::Vector2d pathPoint(double distance) const;
    
    double pathYaw(double distance, double fallback_yaw) const;

    PlannerParams param_;
    std::shared_ptr<path_plan::SearchInput> search_input_;
    std::unique_ptr<path_plan::SearcherInterface> searcher_;
    std::shared_ptr<traj_opt::OptimizerInput> optimizer_input_;
    std::unique_ptr<traj_opt::OptimizerInterface> optimizer_;
    std::shared_ptr<const robot_model::RobotModel> robot_;
    std::shared_ptr<const map::EsdfMap2D> esdf_;
    std::vector<Eigen::Vector2d> global_path_;
    std::vector<double> path_lengths_;
    Eigen::Vector3d global_goal_ = Eigen::Vector3d::Zero(); // (x, y, yaw)
    poly_traj::Trajectory<3> previous_trajectory_;
    double previous_start_time_ = 0, previous_aim_distance_ = 0;
    bool force_search_ = false;
    MotionPhase phase_ = MotionPhase::NewPath;
    double alignment_yaw_ = 0;
};
} // namespace navapex::planner
