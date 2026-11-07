#pragma once

#include <memory>
#include <optimizer/base_optimizer.hpp>
#include <params/base_params.hpp>
#include <optimizer_io.hpp>
#include <string>

namespace navapex::traj_opt
{
class OptimizerInterface
{
public:
    OptimizerInterface(const std::string &type, std::shared_ptr<const OptimizerInput> input);

    OptimizerResult optimize();

    const OptimizerParams &param() const;

private:
    std::string type_;
    std::shared_ptr<const OptimizerInput> input_;
    std::unique_ptr<Optimizer> optimizer_;
};
} // namespace navapex::traj_opt
