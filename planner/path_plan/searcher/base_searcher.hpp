#pragma once

#include <Eigen/Core>
#include <vector>

namespace path_plan
{
class SearcherParams;

struct Result
{
    bool success = false;
    std::vector<Eigen::Vector3d> path;
    double total_cost = 0; // 路径长度，单位米
    double elapsed_ms = 0;
};

class Searcher
{
public:
    virtual ~Searcher() = default;

    virtual Result search() = 0;

    virtual const SearcherParams &param() const = 0;
};
} // namespace path_plan
