# NavApex

路径搜索与轨迹优化库，核心无 ROS 依赖。
持续开发中 11.15之前完善

配置只在所属模块下维护：

- `robot_model/config/robot.json`：机器人形状和尺寸。
- `perception/config/lidar2d.json`：感知与局部地图参数。
- `planner/path_plan/config/astar.json`：路径搜索参数。
- `planner/traj_opt/config/`：轨迹优化参数。
- `planner/config/diff.json`：规划与控制参数。
- `ros2_test/config/navapex.yaml`：ROS 接入与仿真参数。
- `ros2_test/config/global_map.json`：仿真全局地图参数，独立于感知配置。

核心模块自行读取本模块源目录的配置，不支持外部配置路径注入。修改核心 JSON 后重启生效；仿真通过 `./ros2_test/run.sh` 更新地图和 ROS 配置后启动。
