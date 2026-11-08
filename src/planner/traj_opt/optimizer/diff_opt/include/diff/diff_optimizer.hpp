#pragma once

#include <diff/diff_types.hpp>
#include <optimizer/base_optimizer.hpp>
#include <params/diff_params.hpp>

namespace navapex::traj_opt
{
class Diff_Optimizer final : public Optimizer
{
public:
    Diff_Optimizer(DiffParams param, std::shared_ptr<const OptimizerInput> input);
    Diff_Optimizer(const Diff_Optimizer &) = delete;
    Diff_Optimizer &operator=(const Diff_Optimizer &) = delete;

    OptimizerResult optimize() override;

    const DiffParams &param() const override;

private:
    bool initializePlanning();

    bool runOptimization();

    bool solveLbfgs(Eigen::VectorXd &variables);

    bool CheckTrajectory();

    bool evaluateCost(Eigen::Ref<const Eigen::VectorXd> variables,
                      Eigen::Ref<Eigen::VectorXd> gradient);

    void addJerkCost(Eigen::VectorXd &time_gradient);

    void addPenaltyCosts(Eigen::VectorXd &time_gradient); // 包括避障 集群避障 运动学可行性

    void addSpacingCost(Eigen::VectorXd &time_gradient);

    bool decodeVariables(Eigen::Ref<const Eigen::VectorXd> variables);

    void propagateGradient(Eigen::Ref<const Eigen::VectorXd> variables,
                           Eigen::VectorXd &time_gradient,
                           Eigen::Ref<Eigen::VectorXd> gradient);

    int checkEnd() const;

    // 提供给lbfgs的回调函数
    static double costFunctionCallback(void *instance, const double *variables, double *gradient, int size);
    static int progressCallback(void *instance, const double *, const double *, double, double, double, double, int, int, int);

    DiffParams param_;
    DiffPlanningState state_;

    poly_traj::MinJerkOpt<3> minco_;
};
} // namespace navapex::traj_opt
