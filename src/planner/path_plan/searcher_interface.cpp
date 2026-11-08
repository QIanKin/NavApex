#include "searcher_interface.hpp"

#include <astar/astar.hpp>
#include <filesystem>
#include <utility>

namespace navapex::path_plan
{
SearcherInterface::SearcherInterface(const std::string &type,
                                     std::shared_ptr<const SearchInput> input) : input_(std::move(input))
{
    auto path = std::filesystem::path(__FILE__).parent_path() / "config" / "astar.json";
    if(type == "astar3d"){
        searcher_ = std::make_unique<AStar>(true, AStarParams(path.string()), input_);
    }else if (type == "astar2d") {
        searcher_ = std::make_unique<AStar>(false, AStarParams(path.string()), input_);
    } 
}

SearchResult SearcherInterface::search()
{
    return searcher_->search();
}

const SearcherParams &SearcherInterface::param() const
{
    return searcher_->param();
}
} // namespace navapex::path_plan
