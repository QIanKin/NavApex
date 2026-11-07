#pragma once

#include <omni3d/omni_3d_types.hpp>
#include <omni3d/local_astar.hpp>
#include <optimizer/base_optimizer.hpp>
#include <params/omni_3d_params.hpp>

namespace navapex::traj_opt
{
class Omni_3d_Optimizer final : public Optimizer
{
public:
    Omni_3d_Optimizer(Omni3DParams param, std::shared_ptr<const OptimizerInput> input);
    Omni_3d_Optimizer(const Omni_3d_Optimizer &) = delete;
    Omni_3d_Optimizer &operator=(const Omni_3d_Optimizer &) = delete;

    OptimizerResult optimize() override;

    const Omni3DParams &param() const override;

private:
    bool initializePlanning();

    bool runOptimization();

    bool solveLbfgs(Eigen::VectorXd &variables);

    bool CheckTrajectory();

    bool evaluateCost(Eigen::Ref<const Eigen::VectorXd> variables, 
                      Eigen::Ref<Eigen::VectorXd> gradient);

    void addJerkCost(Eigen::VectorXd &time_gradient);
    
    void addPenaltyCosts(Eigen::VectorXd &time_gradient);  // 包括避障 集群避障 运动学可行性
    
    void addSpacingCost(Eigen::VectorXd &time_gradient);
    
    bool decodeVariables(Eigen::Ref<const Eigen::VectorXd> variables);

    void propagateGradient(Eigen::Ref<const Eigen::VectorXd> variables,
                           Eigen::VectorXd &time_gradient, 
                           Eigen::Ref<Eigen::VectorXd> gradient);

    bool collectCollisionSegments(const poly_traj::Trajectory<3> &trajectory,
                                  const Eigen::Matrix3Xd &points, 
                                  std::vector<std::pair<int, int>> &segments, int end);
        
    bool finelyBuildObstacleConstraints(const Eigen::Matrix3Xd &points,
                                        std::vector<std::pair<int, int>> segments);

    void appendObstacleDirections(const Eigen::Matrix3Xd &points,
                                  const std::vector<Eigen::Vector3d> &path, 
                                  const std::pair<int, int> &segment,
                                  const std::pair<int, int> &propagation_segment, 
                                  bool handle_short_segment);

    void roughlyCheckConstraintPoints();

    bool allowRebound() const;
    
    int checkEnd() const;
    
    // 提供给lbfgs的回调函数
    static double costFunctionCallback(void *instance, const double *variables, double *gradient, int size);
    static int progressCallback(void *instance,const double *,const double *,double,double,double,double,int,int,int);

    Omni3DParams param_;
    Omni3DPlanningState state_;

    poly_traj::MinJerkOpt<3> minco_;
    LocalAStar local_search_;
};
} // namespace navapex::traj_opt
