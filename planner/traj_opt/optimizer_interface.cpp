#include "optimizer_interface.hpp"

#include <diff/diff_optimizer.hpp>
#include <filesystem>
#include <omni3d/omni_3d_optimizer.hpp>
#include <tools/log.hpp>

namespace navapex::traj_opt
{
static constexpr char OPTIMIZER_TAG[] = "optimizer";

OptimizerInterface::OptimizerInterface(const std::string &type, 
                                       std::shared_ptr<const OptimizerInput> input) 
                                       : type_(type), input_(std::move(input))
{
    auto path = std::filesystem::path(__FILE__).parent_path() / "config" / (type + ".json");
    if (type == "omni3d")
        optimizer_ = std::make_unique<Omni_3d_Optimizer>(Omni3DParams(path.string()), input_);
    else if(type == "diff")
        optimizer_ = std::make_unique<Diff_Optimizer>(DiffParams(path.string()), input_);
}

OptimizerResult OptimizerInterface::optimize()
{
    if (input_->inner_points.cols() + 1 != input_->durations.size())
    {
        LOGE(OPTIMIZER_TAG) << "init dimension mismatch"
                            << " points=" << input_->inner_points.cols()
                            << " durations=" << input_->durations.size();
        return {};
    }

    if (type_ == "omni3d")
    {
        if (!input_->map)
        {
            LOGE(OPTIMIZER_TAG) << "input map is null";
            return {};
        }
        if (input_->map->space() != perception::Space::Spatial)
        {
            LOGE(OPTIMIZER_TAG) << "omni3d requires a spatial map";
            return {};
        }
    }

    if (type_ == "diff")
    {
        if (!input_->esdf || !input_->robot_)
        {
            LOGE(OPTIMIZER_TAG) << "diff requires esdf and robot";
            return {};
        }
    }
    
    return optimizer_->optimize();
}

const OptimizerParams &OptimizerInterface::param() const
{
    return optimizer_->param();
}

} // namespace navapex::traj_opt
