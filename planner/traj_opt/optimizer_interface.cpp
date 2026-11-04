#include "optimizer_interface.hpp"

#include <cmath>
#include <diff/diff_optimizer.hpp>
#include <filesystem>
#include <omni3d/omni_3d_optimizer.hpp>
#include <tools/log.hpp>

namespace traj_opt
{
static constexpr char OPTIMIZER_TAG[] = "optimizer";

OptimizerInterface::OptimizerInterface(const std::string &type, 
                                       std::shared_ptr<const PlanInput> input) : type_(type), input_(std::move(input))
{
    if (type != "omni3d" && type != "diff"){
        throw std::invalid_argument("Unknown optimizer type: " + type);
    }
    if (!input_)
        throw std::invalid_argument("PlanInput must not be null");
    const auto path = std::filesystem::path(__FILE__).parent_path() / "config" / (type + ".json");
    if (type == "omni3d")
        optimizer_ = std::make_unique<omni3d::Omni_3d_Optimizer>(Omni3DParams(path.string()), input_);
    else
        optimizer_ = std::make_unique<diff::Diff_Optimizer>(DiffParams(path.string()), input_);
}

Result OptimizerInterface::optimize()
{
    double resolution;
    if (type_ == "omni3d")
    {
        if (!input_->map)
        {
            LOGE(OPTIMIZER_TAG) << "input map is null";
            return {};
        }
        if (input_->map->space() != map::Space::Spatial)
        {
            LOGE(OPTIMIZER_TAG) << "omni3d requires a spatial map";
            return {};
        }
        resolution = input_->map->resolution();
    }
    else
    {
        if (!input_->esdf || !input_->robot_model)
        {
            LOGE(OPTIMIZER_TAG) << "diff requires esdf and robot_model";
            return {};
        }
        resolution = input_->esdf->resolution();
    }
    if (!std::isfinite(resolution) || resolution <= 0)
    {
        LOGE(OPTIMIZER_TAG) << "invalid " << type_ << " map resolution=" << resolution;
        return {};
    }
    if (input_->inner_points.cols() + 1 != input_->durations.size())
    {
        LOGE(OPTIMIZER_TAG) << "init dimension mismatch"
                            << " points=" << input_->inner_points.cols()
                            << " durations=" << input_->durations.size();
        return {};
    }
    return optimizer_->optimize();
}

const OptimizerParams &OptimizerInterface::param() const
{
    return optimizer_->param();
}
} // namespace traj_opt
