#pragma once

#include "base_params.hpp"

namespace navapex::traj_opt
{
class Omni3DParams : public OptimizerParams
{
public:
    // 运动限制
    double max_jerk = 20;

    // 多机器人避障
    double swarm_vertical_scale = 2;
    double swarm_horizontal_scale = 1;

    // 迭代与搜索
    int max_rebounds = 20;
    int search_cells = 100;
    double search_timeout_ms = 200;

    explicit Omni3DParams(const std::string &path = {}) : OptimizerParams(path)
    {
        read();
    }

    void read() override
    {
        readBase();
        max_rebounds = json_.value("max_rebounds", max_rebounds);
        search_cells = json_.value("search_cells", search_cells);
        search_timeout_ms = json_.value("search_timeout_ms", search_timeout_ms);
        if (json_.contains("limits"))
        {
            const auto &v = json_.at("limits");
            max_jerk = v.value("jerk", max_jerk);
        }
        if (json_.contains("swarm"))
        {
            const auto &v = json_.at("swarm");
            swarm_vertical_scale = v.value("vertical_scale", swarm_vertical_scale);
            swarm_horizontal_scale = v.value("horizontal_scale", swarm_horizontal_scale);
        }
    }
};
} // namespace navapex::traj_opt
