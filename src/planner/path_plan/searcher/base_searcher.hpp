#pragma once

#include <planner/path_plan/searcher_io.hpp>

namespace navapex::path_plan
{
class SearcherParams;

class Searcher
{
public:
    virtual ~Searcher() = default;

    virtual SearchResult search() = 0;

    virtual const SearcherParams &param() const = 0;
};
} // namespace navapex::path_plan
