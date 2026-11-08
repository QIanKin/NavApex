# NavApex

路径搜索与轨迹优化库，核心无 ROS 依赖。
持续开发中 11.15之前完善

核心源码和模块配置位于 `src/`，`ros2_test/` 为独立的 ROS 2 接入包。
g3log 通过 Git submodule 从官方仓库 `KjellKod/g3log` 引入，目录为
`src/tools/g3log`，固定到官方发布版 `2.6.2`
（提交 `f7014e42e48d52db0c4c973658a8027dbf7a576e`）。
首次克隆可使用 `git clone --recurse-submodules`；
已有仓库在构建前初始化子模块：

```bash
git submodule update --init --recursive
```

在项目根目录构建核心库并运行规划器测试：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DNAVAPEX_BUILD_PLANNER_TESTS=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

配置只在所属模块下维护：

- `src/robot_model/config/robot.json`：机器人形状和尺寸。
- `src/perception/config/lidar2d.json`：感知与局部地图参数。
- `src/planner/path_plan/config/astar.json`：路径搜索参数。
- `src/planner/traj_opt/config/`：轨迹优化参数。
- `src/planner/config/diff.json`：规划与控制参数。
- `ros2_test/config/navapex.yaml`：ROS 接入与仿真参数。
- `ros2_test/config/global_map.json`：仿真全局地图参数，独立于感知配置。

核心模块自行读取本模块源目录的配置，不支持外部配置路径注入。修改核心 JSON 后重启生效；仿真通过 `./ros2_test/run.sh` 更新地图和 ROS 配置后启动。
