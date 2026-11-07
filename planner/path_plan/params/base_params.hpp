#pragma once

#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <string>

namespace navapex::path_plan
{
class SearcherParams
{
public:
    double timeout_ms = 200;
    int max_nodes = 2000000;

    explicit SearcherParams(const std::string &path = {})
    {
        if (path.empty()) return;
        std::ifstream stream(path);
        if (!stream) throw std::invalid_argument("Cannot open config: " + path);
        stream >> json_;
    }
    virtual ~SearcherParams() = default;
    virtual void read() = 0;

protected:
    void readBase()
    {
        if (json_.contains("search"))
        {
            const auto &v = json_.at("search");
            timeout_ms = v.value("timeout_ms", timeout_ms);
            max_nodes = v.value("max_nodes", max_nodes);
        }
    }

    nlohmann::json json_ = nlohmann::json::object();
};
} // namespace navapex::path_plan
