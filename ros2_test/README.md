# ROS 2 差速仿真

ROS 2 Humble + Gazebo Classic + TurtleBot3 Burger。感知、地图、全局搜索、轨迹优化和控制均使用 NavApex；本包不依赖 Nav2，不启动 SLAM。

## 启动

在项目根目录执行，一次完成编译、Gazebo 和 RViz 启动：

```bash
./ros2_test/run.sh
```

在 RViz 顶部选择 **2D Goal Pose**，点击并拖动设置位置和朝向。自动测试预设目标 `(3.5, 0, 0)`：

```bash
./ros2_test/run.sh auto_start:=true
```

无界面运行：

```bash
./ros2_test/run.sh gui:=false rviz:=false auto_start:=true
```

安装依赖为 `gazebo_ros`、`gazebo_plugins`、`turtlebot3_gazebo`、`robot_state_publisher`、`rviz2`，以及核心 C++ 的 Eigen、OpenCV、nlohmann_json。使用系统 Python，避免 Conda 的 Python 干扰 ROS。

## 数据流

```text
world 静态 box → 编译时生成地图 JSON → GridMap2D → 全局 A*
                                                     ↓ 全局路径
Gazebo /scan → LaserScan2D + 对应时刻 RobotState → PerceptionInterface
                                                     ↓ GridMap2D 共享指针
                                               Planner::planLocal
                                                     ↓
                                     局部 ESDF → 轨迹优化 → v/w
                                                     ↓
                                         /cmd_vel → Gazebo
```

`src/navapex_node.cpp` 负责 ROS 数据转换、调度和发布。`PerceptionInterface` 与 `Planner` 共用外部注入的机器人模型；感知结果中的地图指针直接交给规划器，没有 ROS costmap 中转，也没有重新复制一份用于规划的栅格。

`/odom` 使用 Gazebo 插件的 **WORLD 真值模式**（`odometry_source=1`），不是轮速积分。位姿处于世界坐标系，速度转换到车体坐标系；`map → odom` 为恒等变换。插件的实现见 [GazeboRosDiffDrive](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)。加速度由相邻真值速度差分估计，并补上差速转弯的向心分量。

状态发布 200 Hz，激光 20 Hz。节点保留短时间的状态队列，为每帧激光选择时间最近的状态，保留原始时间戳；允许的时间差和输入有效期仍由感知参数决定。当前只完成时间匹配，没有实现逐束运动畸变矫正。

仿真机器人 SDF 在编译目录生成，不修改系统 TurtleBot3 模型。雷达关闭测距噪声，量程 8 米；外参为 `x=-0.032, y=0, yaw=0`。无回波按最大量程清空射线。局部地图随机器人滚动。当前采用导航障碍图策略：`track_unknown_space=false`，初始格子、reset 后的格子及滚动新增格子默认空闲；未观测不会自动生成 255。已标记的障碍仍由后续射线观测清除，不会每帧重置为空闲。只过滤打到自身车体的回波，不强制清空整个车体区域。设置为 `true` 可保留严格未知策略；地图外查询仍返回未知。

全局地图由 `scripts/generate_map.py` 从同一个 `worlds/navapex.world` 的静态 box 碰撞几何生成，分辨率取感知配置（当前 0.05 米），范围 `[-4,6] × [-4,4]`。JSON 的 `costs` 从左下角逐行排列，空闲为 0，实体障碍为 254；从同一份感知配置读取 `forbidden_radius=0.05`，在原始致命格外围生成一格 253 禁行区，包括对角邻格，不递归膨胀。当前生成器面向这个平面、静态 box 场景；更换世界边界、几何类型或增加动态物体时需要相应扩展，动态障碍仍由雷达感知。

全局 A* 使用地图和机器人半径检查通行；局部 A* 仍是点搜索。两者都通过 `occupied()` 拒绝 253、254、255。局部规划收到感知地图后才生成并缓存 ESDF：只有 254、255 和地图外边框作为距离场障碍，253 与衰减代价均不作为种子，机器人模型再减去自身半径得到净距。禁行圈只影响搜索，不重复加到优化器的机器人半径上。

## costmap 可视化

RViz 已配置 **Rolling Local Costmap**，订阅 `/navapex/local_costmap`，类型为 `nav_msgs/OccupancyGrid`，使用 `costmap` 配色。未发送目标时也会持续显示感知地图。

| 核心原始代价 | ROS 显示值 | 含义 |
| --- | --- | --- |
| 0 | 0 | 空闲 |
| 1–252 | 1–98 | 衰减代价 |
| 253 | 99 | 配置的禁行圈 |
| 254 | 100 | 实体障碍 |
| 255 | -1 | 未知 |

转换只用于显示，规划器始终查询原始 0–255 代价。显示消息的原点跟随滚动窗口，时间戳取感知结果时间。`Static Map` 单独显示已知全局地图 `/map`。

| 话题 / 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 激光输入 |
| `/odom` | `nav_msgs/Odometry` | Gazebo 真值位姿与速度 |
| `/map` | `nav_msgs/OccupancyGrid` | 已知全局地图，保留最后一帧 |
| `/navapex/local_costmap` | `nav_msgs/OccupancyGrid` | 感知地图，保留最后一帧 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 目标位置与朝向 |
| `/navapex/global_path` | `nav_msgs/Path` | 蓝色全局路径 |
| `/navapex/initial_path` | `nav_msgs/Path` | 橙色初始多项式 |
| `/navapex/local_path` | `nav_msgs/Path` | 绿色优化轨迹 |
| `/navapex/lookahead` | `geometry_msgs/PoseStamped` | 局部目标 |
| `/navapex/status` | `std_msgs/String` | 输入、规划和到达状态 |
| `/cmd_vel` | `geometry_msgs/Twist` | 线速度 x、角速度 z |
| `/navapex/cancel` | `std_srvs/Trigger` | 取消并停车 |

规划线程与控制发布分离：规划请求 10 Hz，控制发布 20 Hz。规划失败、输入过期、计算超时或目标切换后，不执行旧的非零控制。取消后可手动发布速度：

```bash
ros2 service call /navapex/cancel std_srvs/srv/Trigger '{}'
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.15}, angular: {z: 0.3}}'
# 手动控制结束后发送零速度
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

## 配置

ROS 话题、机器人半径和调度频率在 `config/navapex.yaml`；感知的仿真外参及三层代价在 `config/perception/lidar2d.json`。`forbidden_radius` 和膨胀 `radius` 独立配置，不从机器人半径推导。当前分辨率与禁行半径均为 0.05 米，全局和局部使用相同的半格对角线距离补偿，因此对应八邻域的一格误差余量；软膨胀半径仍为 0.65 米。关闭软膨胀也可独立保留禁行圈。

优化器使用 `config/traj_opt/diff.json`；规划和搜索配置在构建时从核心目录安装到 `config/planner`、`config/path_plan`。launch 设置 `NAVAPEX_CONFIG_DIR`，可通过 `config_dir:=...` 替换整个配置目录。修改源配置或 world 后重新执行启动脚本即可重新安装。

## 参数核对

以实际安装并加载的 `models/burger.sdf` 的碰撞几何为准，计算到 `base_footprint` 的平面包络；不把模型可视网格的外观尺寸作为碰撞尺寸。

| 项目 | 核对结果 |
| --- | --- |
| 底盘 box | 0.140 × 0.140 米，中心 x=-0.032 米 |
| 包含轮子的 XY 范围 | x≈[-0.102, 0.038]，y≈[-0.0890, 0.0890] 米 |
| 关于 base_footprint 的最小外接圆半径 | 约 0.123709 米 |
| 实际注入的 CircleModel | 0.125 米，同一实例给感知和规划；原先 0.18 米偏大 |
| 轮距 / 轮径 | 0.160 / 0.066 米，与 SDF 轮子碰撞几何一致 |
| 轮缘加速度上限 | 保留系统 SDF 的 1.0 m/s²；组合控制需求为 0.4+0.08×1.5=0.52 m/s² |
| 雷达外参 | x=-0.032、y=0、yaw=0，与 SDF 一致；二维感知忽略高度 |
| 雷达 | 360 束，0～6.28 rad，0.12～8 米，20 Hz，测距噪声为 0 |
| 雷达角度、量程、时间字段 | 直接取实际 `/scan`，不在感知 JSON 中重复定义；当前 time_increment=0 |
| 测距分辨率 / 地图分辨率 | 0.015 / 0.05 米，分别属于传感器与栅格，不要求相等 |
| 局部地图 | 160×160，8×8 米；初始和滚动新格默认空闲 |
| 全局地图 | 200×160，10×8 米；空闲 29596 格、253 禁行 896 格、254 致命 1508 格 |
| 搜索禁行余量 | 两边 forbidden_radius=0.05 米，对应向外一格 |
| 优化器 ESDF | 只考虑 254、255；忽略 253、1～252 |
| 线速度 / 线加速度 | 控制与优化均为 0.22 m/s、0.4 m/s² |
| 角速度 / 角加速度 | 控制与优化均为 1.0 rad/s、1.5 rad/s² |
| 巡航速度 | 0.18 m/s，低于 0.22 m/s 上限 |
| 调度与时钟 | 规划 10 Hz 对应控制周期 0.1 秒；命令发布 20 Hz；状态名义 200 Hz；物理步长 0.005 秒；统一仿真时钟 |
| 感知时间限制 | pose/scan 差不超过 0.01 秒；输入年龄不超过 0.20 秒 |
| 到达判定 | 位置 0.10 米、朝向 0.10 rad，并确认停稳 |

核心 `planner/config/diff.json` 中的 `robot_radius=0.18` 只是未注入机器人模型时的后备值。本仿真始终从 ROS 的 `robot_radius=0.125` 实例化并注入模型，搜索和 ESDF 净距查询使用的都是 0.125，不使用后备半径。CircleModel 是实际不规则底盘的圆形包络，不能表达完全相同的非圆轮廓。系统 URDF 供 RViz 和 TF 使用，SDF 供物理仿真使用；两者的高度、脚轮形状和惯量不是逐项相同，本次尺寸核对按实际 SDF 的平面碰撞包络进行。

轮缘加速度参数的单位按插件中“关节角速度×轮半径”转换后的线速度更新逻辑核对，见 [GazeboRosDiffDrive](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)。

## 日志与验证

`tools/log.hpp` 的 `LogConfig::level` 默认改为 `Debug`，包括 Release 构建。`LOGD`、`LOGI`、`LOGW`、`LOGE` 均可写入文件；`solver cost` 的 total、smoothness、time、obstacle、obstacle_soft 等项默认可见。文件在 `log_directory` 指定目录，启动脚本默认使用项目的 `log/`。`console` 仍默认关闭，日志文件级别与 ROS 控制台级别是两套设置。

本次通过初始、reset、滚动新增格的未知策略检查，一格禁行圈检查，以及 253 搜索阻挡 / ESDF 忽略、254 与 255 参与 ESDF 的检查。已重新运行无界面 Gazebo 闭环：通过 `/goal_pose` 发送 `(3.5,0,0)`，约 34 秒绕过中央障碍并报告 `goal_reached`，位置约 `(3.458,0.035)`，位置误差约 5.5 cm。该次最大命令线速度约 0.18 m/s，最大角速度约 0.708 rad/s；323 次局部规划成功，没有规划失败。再次发送目标后调用取消服务，已确认发布零速度。日志中已验证 `D/optimizer: solver cost` 输出。
