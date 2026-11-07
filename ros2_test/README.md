# ROS 2 差速仿真

面向 ROS 2 Humble、Gazebo Classic 和 TurtleBot3 Burger。Nav2 只提供地图服务器、
激光障碍层和膨胀层，不启动 planner server、controller server 或导航行为树。
全局 A*、差速轨迹优化及 `v/w` 控制由 NavApex 完成。

## 编译与启动

一键启动 Gazebo GUI 和已配置的 RViz（自动加载环境并单线程增量编译）：

```bash
./ros2_test/run.sh
```

直接观察机器人导航到预设目标：

```bash
./ros2_test/run.sh auto_start:=true
```

也可以在 RViz 中用 **2D Goal Pose** 交互设置目标。下面是分步命令，便于单独编译或无界面运行。

在 NavApex 根目录执行：

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon --log-base ros2_test/log build \
  --base-paths ros2_test --build-base ros2_test/build --install-base ros2_test/install \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source ros2_test/install/setup.bash
ros2 launch navapex_ros2 simulation.launch.py rviz:=true
```

默认关闭 Gazebo GUI 和 RViz；`rviz:=true` 显示 RViz，`gui:=true` 打开 Gazebo GUI。
默认机器人位于 `(0,0,0)`，中央障碍位于 `(1.6,0)`，RViz 中使用 **2D Goal Pose** 发送目标。
快速运行固定目标 `(3.5,0,0)`：

```bash
ros2 launch navapex_ros2 simulation.launch.py auto_start:=true rviz:=true
```

依赖包括 `turtlebot3_gazebo`、`gazebo_ros`、`gazebo_plugins`、`nav2_costmap_2d`、
`nav2_map_server`、`nav2_lifecycle_manager`、`robot_state_publisher` 和 `rviz2`。
生成机器人时显式使用系统 Python，避免 Conda 环境影响 Gazebo 的 Python 依赖。

Gazebo 使用 `worlds/navapex.world`。编译时 `scripts/generate_map.py` 将同一世界中的静态
矩形障碍生成对应的 PGM 地图，范围为 `[-4,6] × [-4,4]`，分辨率 0.05 米。
初期定位使用 Gazebo 里程计，`map → odom` 为恒等变换；暂未接入 AMCL 或 SLAM。
换成真实定位时，应由定位模块接管这条变换，并删除 launch 中的固定变换发布器。

## 接口与可视化

| 话题或服务 | 类型 | 用途 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo 激光输入 |
| `/odom` | `nav_msgs/Odometry` | 位姿与车体速度 |
| `/global_costmap/costmap_raw` | `nav2_msgs/Costmap` | Nav2 全局膨胀地图 |
| `/local_costmap/costmap_raw` | `nav2_msgs/Costmap` | 6 × 6 米滚动局部地图 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 外部或 RViz 目标 |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x=v`、`angular.z=w` |
| `/navapex/global_path` | `nav_msgs/Path` | 蓝色全局几何路径 |
| `/navapex/initial_path` | `nav_msgs/Path` | 橙色初始多项式，包含热启动初值 |
| `/navapex/local_path` | `nav_msgs/Path` | 绿色优化后的局部轨迹 |
| `/navapex/lookahead` | `geometry_msgs/PoseStamped` | 当前局部目标 |
| `/navapex/status` | `std_msgs/String` | 等待输入、规划失败、跟踪、旋转、到达 |
| `/navapex/cancel` | `std_srvs/Trigger` | 取消导航并停车 |

手动控制时先取消正在运行的导航；空闲状态下 NavApex 不持续发布零速度，因此可以直接使用：

```bash
ros2 service call /navapex/cancel std_srvs/srv/Trigger '{}'
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.15}, angular: {z: 0.3}}'
```

结束手动控制后发送零速度：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

控制发布与规划线程分离。规划按 10 Hz 请求，过载时只保留最新请求，不累积任务。
位姿、地图或控制结果超时，规划失败、新目标切换及取消时均不执行旧的非零控制。
没有单周期硬实时保证；查看日志中的 `elapsed_ms`、`esdf_ms` 评估运行频率。
核心结果不携带耗时、错误文本、初值来源或重置原因，这些诊断统一写日志；ROS 状态话题根据成功、到达标志及适配层事件生成状态。

## 地图和 ESDF

`Nav2GridMap` 直接实现核心 `navapex::perception::OccupancyMap`，只读持有 `nav2_msgs/Costmap` 快照，保留每次更新
的原点、分辨率和尺寸。局部 costmap 的 `rolling_window: true` 负责跟随机器人移动；
两张 costmap 均使用 `map` 坐标系，规划器不把机器人坐标误当成地图索引。

使用原始 `0..255` 代价，不混用 `nav_msgs/OccupancyGrid` 的 `-1..100` 编码：
全局与局部搜索将 `253`、`254`、`255` 视为不可走；ESDF 仅从致命障碍 `254` 和未知 `255`
构建，保留软膨胀区域。圆形机器人查询再减去半径，避免重复扣除 Nav2 的膨胀半径。
默认机器人半径 0.18 米；修改时需要同步 Nav2 的 `robot_radius` 与核心 Planner 配置。

ESDF 使用 OpenCV 精确欧氏距离变换，外边界按障碍处理。连续查询使用格心距离的双线性
插值，梯度是同一插值函数的解析导数；圆形机器人对 yaw 的距离梯度为零。
优化使用 `querySmooth()` 的 Catmull–Rom 双三次距离代理与解析梯度，跨格保持 C1 连续，
不单独归一化梯度。A*、起终点和碰撞精查继续使用原始 `query()`，防止插值过冲掩盖碰撞。
优化查询在边框外提供连续的负距离延伸，给越界试探点回退梯度。
地图更新提供新快照时重建 ESDF，相同快照复用缓存。它是离散栅格距离场，不是解析形状距离。

## 滚动规划与热启动

`navapex::planner::Planner("diff")` 通过 `SearcherInterface`、`OptimizerInterface` 组织输入，核心库不依赖 ROS。
`planGlobal()` 返回并保存 `(x,y,yaw)` 全局几何路径；`planLocal()` 接受当前地图和通用 `navapex::RobotState`，返回多项式轨迹与 `v/w`。
状态统一携带 `Pose3D`、直接成员 `twist/acceleration` 和采样时间 `stamp`；同一 Planner 实例的调用应串行执行。
ROS 适配层使用里程计消息的时间戳，位姿、速度来自实时里程计，前向加速度和角加速度由相邻里程计估计，横向运动加速度按差速运动的 `v*ω` 计算。

第一帧从当前位置沿全局路径截取固定前瞻距离，生成多项式初值交给优化器。

后续保存上一条成功轨迹及其起始时间。按已执行时间裁去前段，对剩余多项式重新采样，
尾部按全局路径补到新前瞻点，保留时间尺度；边界使用当前测量的位姿、速度和加速度。
这采用 `df_a/plan_manage/src/planner_manager.cpp::computeInitState` 的时间重采样思路。

正常滚动时持续使用热启动，不因跟踪误差、前瞻变化或外层碰撞检查主动丢弃。
优化器内部最多尝试三次，沿用更新后的求解变量；失败后才调用一次 A* 到全局前瞻点，
再调用一次优化器（内部最多三次）。仍失败则停车，后续周期使用最新输入继续恢复。
内部数值求解失败仍按原有机制立即返回。旧轨迹耗尽或时钟回退时重新使用全局切片。
新全局目标、`reset()` 或外部调用 `discardWarmStart()` 会清除旧初值。

轨迹动态限值与 ESDF 圆形包络碰撞检查统一由差速优化器的 `CheckTrajectory()` 完成。
差速对整条局部前瞻轨迹施加静态和邻机避障代价；三维全向按 `df_a` 仅对前约 2/3 采样点施加这些代价，终点阶段覆盖全段。
普通滚动轨迹只验收前约 2/3 的约束点对应时间范围，终点阶段验收全段，包含验收边界点。
`touches_goal` 仅表示本次局部目标选中了导航终点，初始原地旋转不设置该标志；
终点位置到达后的最终朝向对齐属于终点阶段。差速的该标志不裁剪避障代价范围，
机器人实际到达并停稳由独立的 `goal_reached` 表达。
差速的 `checkEnd()` 用于确定轨迹及邻机的验收范围，控制器也只在已验收范围内取追踪点。
地图外查询延伸为负距离和指向地图内的梯度，允许优化器回退越界试探；碰撞精查仍拒绝地图外包络。
期望障碍间距继续由障碍代价处理。
避障激活距离均以机器人外壳到实体障碍的净距计算：净距 = ESDF 距离 - 机器人半径。
当前硬避障罚项在净距小于 0.10 m 时激活，软避障在小于 0.30 m 时激活。
圆形包络半径保持 0.18 m，与 Nav2 一致；因此圆心距离阈值分别为 0.28 m 和 0.48 m。
硬避障权重为 100000，软避障权重为 5000；硬避障项仍是罚函数，实体穿入由精查否决。
全局和局部 A* 直接使用输入栅格，不生成或查询 ESDF；253、254、255 均不可通行。
全局 A* 使用机器人半径做圆形栅格检查，检查起终点、节点和连线采样点；局部 A* 保持点搜索，车体碰撞由后续轨迹优化和验收处理。
二维局部优化时才生成并缓存 ESDF，机器人模型结合位姿返回净距和梯度。
查询返回 `void`，通过 `double &distance` 输出距离，需要梯度时使用额外带 Eigen 向量参数的重载。轨迹验收只查距离并判断净距是否小于零；优化代价查询距离及梯度，梯度实时计算。
实际端点空闲但格心碰撞时，仅在相邻格寻找无碰撞的直连格心。
动态验收的四项绝对容忍均为 0.5，优化限值和实际控制限幅保持各自配置。
不增加 via point 代价或每帧 A*，历史轨迹提供绕障方向的连续性。

显式原地转向只用于两个阶段：新全局路径与当前朝向偏差超过 `local.rotate_threshold`
（默认约 60 度）时先停稳再对齐；到达终点位置后先停稳再对齐最终 yaw。
停稳依据实测线速度、角速度及持续时间确认，A* 恢复不触发初始转向。
行进阶段的局部目标 yaw 使用路径切线，中间目标保留前进速度，真正终点末端速度为零。
最终目标 yaw 独立保存，不写入行进路径末点。

日志的 `seed=global_slice/warm_start/astar/rotation` 和 `reset=` 可观察初值来源。
状态区分 `tracking`、`initial_braking/initial_rotating`、`goal_braking/goal_rotating`、`goal_reached`。

## 配置

ROS 参数位于 `config/navapex.yaml`、`config/costmaps.yaml`。核心参数仍从 JSON 加载：
`config/planner/diff.json`、`config/path_plan/astar.json`、`config/traj_opt/diff.json`。
launch 设置 `NAVAPEX_CONFIG_DIR` 指向安装后的配置目录，也可以通过 `config_dir:=...` 指定。
原始核心文件的默认配置仍保留在源码对应目录中；没有增加参数重载构造函数。
修改源码中的配置后重新编译安装即可生效。

实现使用 Nav2 Humble 的 [Costmap2DROS 接口](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_ros.cpp)
和 [代价地图配置](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)。
自带 `nav2_costmap_2d` 可执行程序固定使用字符串构造，本包的薄入口使用 NodeOptions 构造，
让两个地图实例正确接收各自的命名空间、节点名和 YAML 参数。
