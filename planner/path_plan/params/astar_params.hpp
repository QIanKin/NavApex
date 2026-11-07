#pragma once

#include "base_params.hpp"

namespace navapex::path_plan
{
class AStarParams : public SearcherParams
{
public:
    explicit AStarParams(const std::string &path = {}) : SearcherParams(path) { read(); }

    void read() override
    {
        readBase();
    }
};
} // namespace navapex::path_plan
