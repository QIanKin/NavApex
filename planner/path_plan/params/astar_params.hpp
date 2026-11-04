#pragma once

#include "base_params.hpp"

namespace path_plan
{
class AStarParams : public SearcherParams
{
public:
    AStarParams() = default;

    explicit AStarParams(const std::string &path)
    {
        const auto json = readJson(path);
        readCommon(json);
        validate();
    }
};
} // namespace path_plan
