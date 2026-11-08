#pragma once

#include <memory>
#include <planner/path_plan/params/base_params.hpp>
#include <planner/path_plan/searcher_io.hpp>
#include <planner/path_plan/searcher/base_searcher.hpp>
#include <string>

namespace navapex::path_plan
{
class SearcherInterface
{
public:
    SearcherInterface(const std::string &type, std::shared_ptr<const SearchInput> input);

    SearchResult search();

    const SearcherParams &param() const;

private:
    std::shared_ptr<const SearchInput> input_;
    std::unique_ptr<Searcher> searcher_;
};
} // namespace navapex::path_plan
