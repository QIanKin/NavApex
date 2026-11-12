#pragma once

#include <nlohmann/json.hpp>
#include <perception/maps/base_map.hpp>
#include <planner/plan_manage/planner_io.hpp>
#include <tools/log.hpp>

// ROS 无关的日志序列化层。地图必须是不可变快照，编码和压缩在日志线程执行。
namespace navapex::replay
{
using Json = nlohmann::json;
bool enabled() noexcept;
Json state(const RobotState &state);
void event(Json fields) noexcept;
void map(Json fields, std::shared_ptr<const perception::OccupancyMap> map) noexcept;
void plan(Json fields, const planner::LocalPlanResult &result,
          const std::vector<Position3D> &global_path) noexcept;
void health(double stamp, std::uint64_t clock_epoch = 0) noexcept;
} // namespace navapex::replay
