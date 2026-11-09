# NavApex 本地日志播放器

在项目根目录运行（仅需 Python 3.10+ 标准库，无需 ROS、npm 或联网）：

```bash
python3 tools/log_viewer/main.py
# 或启动时直接打开一组日志，包括轮转文件
python3 tools/log_viewer/main.py log/replay_acceptance/*.log*
```

浏览器会打开 `http://127.0.0.1:8765`。也可使用 `--no-browser --port 8766`。
服务只监听本机，上传文件保存在临时目录，退出时清理；原始日志不修改。

## 录制

ROS 节点默认使用 Debug 诊断等级，并开启结构化回放。启动仿真后即记录定位和感知；发送目标后记录规划。无需录 rosbag。

```bash
./ros2_test/run.sh
```

日志目录默认为项目的 `log/`。`ros2_test/config/navapex.yaml` 支持：

```yaml
log_replay: true
log_max_file_mb: 64
log_max_files: 8
```

默认每个运行会话最多保留 8 × 64 MiB，超过后删除最早的轮转文件。请及时归档需要保留的运行；这是容量限制，不保证固定录制时长。轮转后文件名为 `navapex.…log.1`、`.log.2` 等，**把同一次录制的 `.log*` 一起导入**。顺序不限，重复导入的结构化记录按会话及序号去重。

`log_replay: false` 关闭地图、定位和轨迹回放数据，普通 Debug 日志仍保留。

## 使用与验收

1. 选择或拖入日志，左上角显示记录的本地时分秒，下方显示源时钟及相对回放时间。
2. 默认为“规划帧”，使用左右按钮或方向键逐帧切换。某帧的机器人位姿和局部地图取自该次规划的实际输入，地图缺失时明确提示，不取邻近帧替代。
3. 空格播放／暂停；进度条可前后跳转；支持 0.25～4 倍速。“全部事件”可查看没有规划时的定位、感知和控制事件。
4. 滚轮缩放、拖动平移；“适配地图”恢复全图。“跟随机器人”固定机器人在视野中心。
5. footprint 使用日志中记录的机器人模型半径。目前为圆形，R=0.125 m；朝向线随定位旋转。对于非圆模型，当前只显示其外接圆，不声称还原非圆轮廓。
6. 可分别开关全局地图、局部 costmap、定位轨迹、全局路径和优化轨迹。定位尾迹显示最近 1000 个采样；鼠标悬停显示世界坐标及原始 cost。
7. 失败或未采用的规划以状态和虚线轨迹标识；查看右侧耗时、输入帧 ID、规划指令与已发布指令。右侧文本按日志写入时间同步，只有毫秒精度的旧文本不能精确区分同一毫秒内先后。
8. 关注底部丢弃／错误计数及黄色提示。缺失地图、截断行、损坏数据会提示；不要把缺少的图层理解为空闲空间。

旧日志仍可查看文本（预览最后 100 行），但没有记录的地图和定位无法恢复，需重新录制。现有旧 `LOG_TRAJECTORY` 文本保留显示；完整图形回放使用新协议的 `plan` 事件。

播放器用于离线回放。正在写入的日志需要重新导入才能看到新增内容；当前不做实时文件跟踪。首版显示感知输出 costmap，不包含原始 LaserScan 点云层。

## 数据协议 v1

沿用现有日志行头：

```text
HH:MM:SS mmm(pid)I/replay: epoch_ns=… seq=… channel=replay … | {JSON}
```

外层日志反斜杠转义先还原一次，然后解析 JSON。`epoch_ns` 是墙上时钟，JSON 的 `stamp` 是源时钟（ROS 仿真中为仿真秒）；二者不能混用。`clock_epoch` 在定位时间倒退超过 0.5 s 时递增，播放器分成独立片段。`seq` 同时被诊断日志占用，不能仅用序号间隔推断丢帧；使用 `health` 的累计计数和引用缺失提示。

每个轮转文件的 `# replay_metadata=` 包含机器人半径、坐标系、规划类型及感知参数。全局地图每 5 秒重发一次检查点；局部地图每次感知更新记录完整快照，可独立解码。

| event | 数据 |
| --- | --- |
| `pose` | `state_id`、源时间、位置、yaw、完整四元数、车体速度 |
| `costmap` | `map_id`、global/local、坐标系、原点、宽高、分辨率、感知使用的状态；原始 uint8 栅格 gzip+Base64 |
| `perception` | 未成功生成地图的感知帧及其状态 ID |
| `plan` | `plan_id`、`goal_id`、精确地图/状态 ID、输入位姿、目标、全局路径、两条多项式、耗时、状态、是否采用、规划指令 |
| `goal` | 目标更新及取消 |
| `command` | ROS 实际发布的线速度、角速度，含超时停车的零指令 |
| `health` | 累计提交、写入、丢弃、错误和当前待写数量 |

栅格布局为 `index = y * width + x`，原点是左下角，分辨率单位米；0 空闲、1～252 代价、253 禁行、254 实体障碍、255 未知。直接保存原始栅格，不经过 ROS 的 -1/0～100 显示编码。

全局路径 `global_path` 只保存几何位置，`global_path_axes=[x,y,z]`，二维规划中 z 为 0；任务终点 yaw 保存在 `goal` 中。旧日志没有 `global_path_axes` 时，第三项可能为 yaw；播放器仅使用全局路径的 x、y 绘制折线。

每条差速轨迹显式声明 `axes=[x,y,yaw]`。每段包含 duration 和 3×6 系数，降幂排列，使用该段局部秒数计算。GUI 对连续多项式采样显示，原始系数保留在日志中。泛用 `LOG_TRAJECTORY` 生产端分别标识差速 `x,y,yaw` 和三维 `x,y,z`。

地图压缩和大轨迹 JSON 编码在现有 g3log sink 线程执行。调用线程只捕获不可变地图和轨迹快照；日志队列维持数量及内存预算，超过预算计入 dropped。后台日志故障不抛入规划求解。

## 验证命令

```bash
python3 -m unittest discover -s tools/log_viewer -p 'test_*.py' -v
node --check tools/log_viewer/dist/app.js  # 可选开发检查，运行播放器不需要 Node

cmake -S . -B build-replay -DCMAKE_BUILD_TYPE=Release \
  -DNAVAPEX_BUILD_REPLAY_TESTS=ON -DNAVAPEX_BUILD_PLANNER_TESTS=ON
cmake --build build-replay -j2
ctest --test-dir build-replay --output-on-failure
# C++ 序列化测试产生的演示日志，包含失败帧及未采用帧
python3 tools/log_viewer/main.py build-replay/src/replay-fixture/*.log*
```

测试覆盖全范围栅格无损还原、后台编码、队列/编码错误计数、轮转元数据、重复/乱序导入、精确地图关联、前后跳帧一致、多项式分段时间、时间重置、损坏/截断数据、旧文本及 HTTP 上传/解码。
