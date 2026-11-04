#pragma once

#include <memory>
#include <optimizer/base_optimizer.hpp>
#include <params/base_params.hpp>
#include <plan_input.hpp>
#include <string>

namespace traj_opt
{
class OptimizerInterface
{
public:
    OptimizerInterface(const std::string &type, std::shared_ptr<const PlanInput> input);

    Result optimize();

    const OptimizerParams &param() const;

private:
    std::string type_;
    std::shared_ptr<const PlanInput> input_;
    std::unique_ptr<Optimizer> optimizer_;
};
} // namespace traj_opt
