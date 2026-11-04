#pragma once

#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace path_plan
{
struct SearchConfig
{
    int search_cells = 100; // 起终点中点附近的搜索窗口，每个有效轴的格数
    double timeout_ms = 200;
};

class SearcherParams
{
public:
    SearchConfig search;

    virtual ~SearcherParams() = default;

    virtual void validate() const
    {
        if (search.search_cells < 5 || search.search_cells > 200 ||
            !std::isfinite(search.timeout_ms) || search.timeout_ms <= 0)
            throw std::invalid_argument("Invalid search window or timeout");
    }

    static nlohmann::json readJson(const std::string &path)
    {
        std::ifstream stream(path);
        if (!stream)
            throw std::invalid_argument("Cannot open config: " + path);
        nlohmann::json json;
        stream >> json;
        return json;
    }

protected:
    void readCommon(const nlohmann::json &json)
    {
        if (json.contains("search"))
        {
            const auto &v = json.at("search");
            search.search_cells = v.value("search_cells", search.search_cells);
            search.timeout_ms = v.value("timeout_ms", search.timeout_ms);
        }
    }
};
} // namespace path_plan
