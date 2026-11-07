#pragma once

#include <cstdlib>
#include <filesystem>
#include <string>

namespace navapex::planner
{
inline std::filesystem::path configPath(const std::string &group, const std::string &name,
                                      const std::filesystem::path &fallback)
{
    if (const char *directory = std::getenv("NAVAPEX_CONFIG_DIR"))
        return std::filesystem::path(directory) / group / name;
    return fallback / name;
}
} // namespace navapex::planner
