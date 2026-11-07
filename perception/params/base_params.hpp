#pragma once

#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <string>

namespace navapex::perception
{
class PerceptionParams
{
public:
    double max_pose_scan_dt = 0.01;
    double max_input_age = 0.20;

    explicit PerceptionParams(const std::string &path = {})
    {
        if (path.empty()) return;
        std::ifstream stream(path);
        if (!stream) throw std::invalid_argument("Cannot open config: " + path);
        stream >> json_;
    }
    virtual ~PerceptionParams() = default;
    virtual void read() = 0;

protected:
    nlohmann::json json_ = nlohmann::json::object();
};
} // namespace navapex::perception
