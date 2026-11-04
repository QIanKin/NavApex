#include "searcher_interface.hpp"

#include <astar/astar.hpp>
#include <filesystem>
#include <utility>

namespace path_plan
{
SearcherInterface::SearcherInterface(const std::string &type,
                                     std::shared_ptr<const SearchInput> input) : type_(type), input_(std::move(input))
{
    if (type != "astar2d" && type != "astar3d")
        throw std::invalid_argument("Unknown searcher type: " + type);
    if (!input_)
        throw std::invalid_argument("SearchInput must not be null");
    const auto path = std::filesystem::path(__FILE__).parent_path() / "config" / "astar.json";
    searcher_ = std::make_unique<AStar>(type == "astar3d", AStarParams(path.string()), input_);
}

Result SearcherInterface::search()
{
    return searcher_->search();
}

const SearcherParams &SearcherInterface::param() const
{
    return searcher_->param();
}
} // namespace path_plan
