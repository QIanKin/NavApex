#pragma once

#include <diff/diff_types.hpp>
#include <optimizer/base_optimizer.hpp>
#include <params/diff_params.hpp>

namespace diff
{
class Diff_Optimizer final : public traj_opt::Optimizer
{
public:
    Diff_Optimizer(traj_opt::DiffParams param, std::shared_ptr<const traj_opt::PlanInput> input);
    Diff_Optimizer(const Diff_Optimizer &) = delete;
    Diff_Optimizer &operator=(const Diff_Optimizer &) = delete;

    traj_opt::Result optimize() override;

    const traj_opt::DiffParams &param() const override;

private:
    bool initializePlanning();

    bool runOptimization();

    bool solveLbfgs(Eigen::VectorXd &variables);

    bool CheckTrajectory();

    bool evaluateCost(Eigen::Ref<const Eigen::VectorXd> variables,
                      Eigen::Ref<Eigen::VectorXd> gradient);

    void addJerkCost(Eigen::VectorXd &time_gradient);

    bool addPenaltyCosts(Eigen::VectorXd &time_gradient); // 包括避障 集群避障 运动学可行性

    void addSpacingCost(Eigen::VectorXd &time_gradient);

    bool decodeVariables(Eigen::Ref<const Eigen::VectorXd> variables);

    void propagateGradient(Eigen::Ref<const Eigen::VectorXd> variables,
                           Eigen::VectorXd &time_gradient,
                           Eigen::Ref<Eigen::VectorXd> gradient);

    bool queryClearance(const Eigen::Vector3d &pose, robot_model::Clearance &result) const;

    int obstacleEnd() const;

    // 提供给lbfgs的回调函数
    static double costFunctionCallback(void *instance, const double *variables, double *gradient, int size);
    static int progressCallback(void *instance, const double *, const double *, double, double, double, double, int, int, int);

    traj_opt::DiffParams param_;
    PlanningState state_;

    poly_traj::MinJerkOpt<3> minco_;
};
} // namespace diff
