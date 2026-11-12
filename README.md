# NavApex

路径搜索与轨迹优化库，核心无 ROS 依赖。
持续开发中 11.15之前完善

核心源码和模块配置位于 `src/`，`ros2_test/` 为独立的 ROS 2 接入包。

日志回放入口：`python3 tools/log_viewer/main.py`，拖入 `.log` 与轮转文件即可查看定位、实时 costmap、机器人 footprint 及规划轨迹。ROS 默认开启 Debug 和回放记录；详见 [本地日志播放器](tools/log_viewer/README.md)。

g3log 通过 Git submodule 从官方仓库 `KjellKod/g3log` 引入，目录为
`src/tools/g3log`，固定到官方发布版 `2.6.2`
（提交 `f7014e42e48d52db0c4c973658a8027dbf7a576e`）。
首次克隆可使用 `git clone --recurse-submodules`；
已有仓库在构建前初始化子模块：

```bash
git submodule update --init --recursive
```

在项目根目录构建核心库：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

完整工作流程测试统一通过 `ros2_test` 运行，覆盖感知、全局规划、局部规划和控制闭环：

```bash
./ros2_test/run.sh
```

在 RViz 中设置目标即可开始导航；无界面运行和参数说明见 [ROS 2 工作流程](ros2_test/README.md)。

配置只在所属模块下维护：

- `src/robot_model/config/robot.json`：机器人形状、尺寸、雷达安装位姿及雷达参数、运动能力上限，以及 `planner_type`（默认 `diff`）。
- `src/perception/config/lidar2d.json`：感知与局部地图参数；雷达安装位姿由机器人模型提供。
- `src/planner/path_plan/config/astar.json`：路径搜索参数。
- `src/planner/traj_opt/config/`：轨迹优化参数。
- `src/planner/plan_manage/config/planner.json`：Planner 自身的局部规划、旋转控制与运动限制参数。
- `ros2_test/config/navapex.yaml`：ROS 接入与仿真参数。
- `ros2_test/config/global_map.json`：仿真全局地图参数，独立于感知配置。

应用入口通过 params 中的 `PlannerConfig::defaultConfig()` 固定加载规划参数，应用初始化根据 RobotModel 的 `planner_type` 选择实现，直接通过 `DiffPlanner(robot, config.planner)` 构造规划器。当前默认 `diff`，使用 `astar2d` 与 `diff` 优化器。
`planner/plan_manage/base_planner.hpp` 定义 `Planner` 抽象接口，`planGlobal()` 和 `planLocal()` 由 `DiffPlanner` 实现，代码位于 `planner/plan_manage/diff_planner.hpp/.cpp`。
全局和局部规划复用同一个搜索器，每次搜索前更新同一份 `SearchInput` 的起点、终点、地图和半径。
配置文件位置由代码固定，配置内容仍按职责分属各模块；修改 JSON 后重启生效。
