#include "optimizer_interface.hpp"

#include <filesystem>
#include <omni3d/omni_3d_optimizer.hpp>

namespace traj_opt
{
OptimizerInterface::OptimizerInterface(const std::string &type, 
                                       std::shared_ptr<const PlanInput> input)
{
    if (type != "omni3d"){
        throw std::invalid_argument("Unknown optimizer type: " + type);
    }
    const auto path = std::filesystem::path("planner/traj_opt/config") / (type + ".json");
    optimizer_ = std::make_unique<omni3d::Omni_3d_Optimizer>(Omni3DParams(path.string()), 
                                                             std::move(input));
}

Result OptimizerInterface::optimize()
{
    return optimizer_->optimize();
}

const OptimizerParams &OptimizerInterface::param() const
{
    return optimizer_->param();
}
} // namespace traj_opt
