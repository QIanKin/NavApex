#include "planner_params.hpp"

#include <filesystem>

namespace navapex::planner
{
PlannerConfig PlannerConfig::defaultConfig()
{
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() / "config" / "planner.json";
    PlannerConfig config;
    config.planner = PlannerParams(path.string());
    return config;
}
} // namespace navapex::planner
