#include <perception/perception_interface.hpp>
#include <perception/maps/grid_map_2d.hpp>
#include <perception/params/lidar_2d_params.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <deque>
#include <planner/planner.hpp>
#include <tools/log.hpp>
#include <tools/replay.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/core.hpp>
#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace navapex::ros2
{
using Steady = std::chrono::steady_clock;
using MapPtr = std::shared_ptr<const perception::OccupancyMap>;

class NavigationNode final : public rclcpp::Node
{
public:
    NavigationNode() : Node("navapex_planner"), tf_(get_clock()), listener_(tf_)
    {
        cv::setNumThreads(1);
        frame_ = declare_parameter("global_frame", "map");
        input_timeout_ = declare_parameter("input_timeout", 0.5);
        map_timeout_ = declare_parameter("map_timeout", 1.0);
        command_timeout_ = declare_parameter("command_timeout", 0.35);
        double planning_frequency = declare_parameter("planning_frequency", 10.0);
        double command_frequency = declare_parameter("command_frequency", 20.0);
        if (!std::isfinite(planning_frequency) || planning_frequency <= 0 ||
            !std::isfinite(command_frequency) || command_frequency <= 0 || input_timeout_ <= 0 ||
            map_timeout_ <= 0 || command_timeout_ <= 0)
            throw std::invalid_argument("Invalid node frequency or timeout");
        planner_tools::LogConfig logging;
        logging.directory = declare_parameter("log_directory", "log");
        logging.replay = declare_parameter("log_replay", true);
        int log_mb = declare_parameter("log_max_file_mb", 64);
        int log_files = declare_parameter("log_max_files", 8);
        if (log_mb <= 0 || log_files <= 0) throw std::invalid_argument("Invalid log retention");
        logging.max_file_bytes = static_cast<std::size_t>(log_mb) * 1024 * 1024;
        logging.max_files = static_cast<std::size_t>(log_files);
        planner_tools::initLog(logging);
        robot_ = robot_model::RobotModel::create();
        planner_ = std::make_unique<planner::Planner>(robot_);
        perception_ = std::make_unique<perception::PerceptionInterface>("lidar2d");
        const auto &lidar = dynamic_cast<const perception::Lidar2DParams &>(perception_->param());
        planner_tools::setReplayMetadata(nlohmann::json({{"v", 1}, {"frame", frame_},
            {"robot", {{"shape", robot_->type()}, {"radius", robot_->radius()}}},
            {"planner_type", planner_->param().type}, {"source_clock", "ros"},
            {"trajectory_axes", {"x", "y", "yaw"}},
            {"perception", {{"resolution", lidar.resolution}, {"width", lidar.width}, {"height", lidar.height},
                {"forbidden_radius", lidar.forbidden_radius}, {"inflation_radius", lidar.inflation_radius},
                {"cost_scaling", lidar.cost_scaling}, {"track_unknown_space", lidar.track_unknown_space},
                {"base_from_laser", {lidar.base_from_laser.x, lidar.base_from_laser.y, lidar.base_from_laser.yaw}}}}}).dump());
        auto map_qos = rclcpp::QoS(1).reliable().transient_local();
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/navapex/local_costmap", map_qos);
        std::ifstream stream(declare_parameter<std::string>("global_map_file", ""));
        nlohmann::json data;
        stream >> data;
        perception::GridGeometry2D geometry;
        geometry.resolution = data.at("resolution");
        geometry.width = data.at("width");
        geometry.height = data.at("height");
        geometry.origin_x = data.at("origin_x");
        geometry.origin_y = data.at("origin_y");
        global_map_ = std::make_shared<perception::GridMap2D>(geometry,
            data.at("costs").get<std::vector<std::uint8_t>>());
        map_pub_->publish(mapMessage(*global_map_, now()));
        replay::map({{"map_id", "global"}, {"layer", "global"}, {"frame", frame_},
                     {"stamp", now().seconds()}, {"clock_epoch", 0}}, global_map_);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(declare_parameter("scan_topic", "/scan"),
            rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message)
        {
            pending_scan_ = std::move(message);
            processScan();
        });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic", "/odom"),
            rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { onOdometry(*message); });
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(declare_parameter("goal_topic", "/goal_pose"),
            10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { onGoal(*message); });
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(declare_parameter("cmd_vel_topic", "/cmd_vel"), 10);
        global_pub_ = create_publisher<nav_msgs::msg::Path>("/navapex/global_path", map_qos);
        local_pub_ = create_publisher<nav_msgs::msg::Path>("/navapex/local_path", 1);
        initial_pub_ = create_publisher<nav_msgs::msg::Path>("/navapex/initial_path", 1);
        lookahead_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/navapex/lookahead", 1);
        status_pub_ = create_publisher<std_msgs::msg::String>("/navapex/status", map_qos);
        cancel_ = create_service<std_srvs::srv::Trigger>("/navapex/cancel",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            has_goal_ = false;
            ++goal_version_;
            replay::event({{"event", "goal"}, {"stamp", now().seconds()}, {"clock_epoch", clock_epoch_},
                           {"goal_id", goal_version_}, {"status", "canceled"}});
            publish_stop_ = true;
            response->success = true;
            response->message = "Navigation canceled; manual cmd_vel control is available";
        });
        double goal_x = declare_parameter("goal_x", 3.5), goal_y = declare_parameter("goal_y", 0.0);
        double goal_yaw = declare_parameter("goal_yaw", 0.0);
        if (declare_parameter("auto_start", false))
        {
            goal_ << goal_x, goal_y, goal_yaw;
            has_goal_ = true;
            ++goal_version_;
            replay::event({{"event", "goal"}, {"stamp", now().seconds()}, {"clock_epoch", clock_epoch_},
                           {"goal_id", goal_version_}, {"goal", {goal_x, goal_y, goal_yaw}}, {"status", "new_goal"}});
        }
        plan_timer_ = create_wall_timer(std::chrono::duration<double>(1 / planning_frequency), [this]
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_goal_) return;
            requested_ = true;
            wake_.notify_one();
        });
        control_timer_ = create_wall_timer(std::chrono::duration<double>(1 / command_frequency), [this] { publishCommand(); });
        replay_timer_ = create_wall_timer(std::chrono::seconds(5), [this]
        {
            replay::health(now().seconds(), clock_epoch_);
            // 静态地图定期作为检查点重发，轮转日志缺少开头时仍可恢复背景。
            replay::map({{"map_id", "global"}, {"layer", "global"}, {"frame", frame_},
                         {"stamp", now().seconds()}, {"clock_epoch", clock_epoch_}}, global_map_);
        });
        worker_ = std::thread([this] { planningLoop(); });
        publishStatus("waiting_for_goal");
        RCLCPP_INFO(get_logger(), "NavApex ready: %g Hz planning; RViz goal topic /goal_pose", planning_frequency);
    }

    ~NavigationNode() override
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
            wake_.notify_one();
        }
        if (worker_.joinable()) worker_.join();
        if (rclcpp::ok()) cmd_pub_->publish(geometry_msgs::msg::Twist());
        try
        {
            planner_tools::flushLog();
            replay::health(now().seconds(), clock_epoch_);
            planner_tools::flushLog();
        }
        catch (...) {}
    }

private:
    struct RecordedState { RobotState state; std::uint64_t id; };
    struct Snapshot
    {
        MapPtr global, local;
        RobotState state;
        Eigen::Vector3d goal;
        std::uint64_t version;
        Steady::time_point captured, odom_received, map_received;
        std::uint64_t state_id = 0, map_id = 0, plan_id = 0, clock_epoch = 0;
    };

    static double age(Steady::time_point timestamp)
    { return std::chrono::duration<double>(Steady::now() - timestamp).count(); }

    geometry_msgs::msg::PoseStamped poseMessage(const Eigen::Vector3d &pose) const
    {
        geometry_msgs::msg::PoseStamped message;
        message.header.frame_id = frame_;
        message.header.stamp = now();
        message.pose.position.x = pose.x();
        message.pose.position.y = pose.y();
        tf2::Quaternion rotation;
        rotation.setRPY(0, 0, pose.z());
        message.pose.orientation = tf2::toMsg(rotation);
        return message;
    }

    nav_msgs::msg::Path pathMessage(const std::vector<Eigen::Vector3d> &poses) const
    {
        nav_msgs::msg::Path message;
        message.header.frame_id = frame_;
        message.header.stamp = now();
        for (const auto &pose : poses) message.poses.push_back(poseMessage(pose));
        return message;
    }

    nav_msgs::msg::Path globalPathMessage(const std::vector<Eigen::Vector3d> &positions) const
    {
        nav_msgs::msg::Path message;
        message.header.frame_id = frame_;
        message.header.stamp = now();
        for (const auto &position : positions)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = message.header;
            pose.pose.position.x = position.x();
            pose.pose.position.y = position.y();
            pose.pose.position.z = position.z();
            pose.pose.orientation.w = 1;
            message.poses.push_back(pose);
        }
        return message;
    }

    void publishStatus(const std::string &text)
    {
        std_msgs::msg::String message;
        message.data = text;
        status_pub_->publish(message);
    }

    void onGoal(const geometry_msgs::msg::PoseStamped &input)
    {
        try
        {
            auto pose = input;
            if (pose.header.frame_id.empty()) pose.header.frame_id = frame_;
            if (pose.header.frame_id != frame_) pose = tf_.transform(pose, frame_, tf2::durationFromSec(0.1));
            Eigen::Vector3d goal(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
            if (!goal.allFinite()) throw std::invalid_argument("Non-finite goal");
            {
                std::lock_guard<std::mutex> lock(mutex_);
                goal_ = goal;
                has_goal_ = true;
                ++goal_version_;
                replay::event({{"event", "goal"}, {"stamp", now().seconds()}, {"clock_epoch", clock_epoch_},
                               {"goal_id", goal_version_}, {"goal", {goal.x(), goal.y(), goal.z()}},
                               {"status", "new_goal"}});
                requested_ = true;
                wake_.notify_one();
            }
            publishStatus("new_goal");
            LOGI("ros2") << "new goal=" << goal.transpose();
        }
        catch (const std::exception &error) { RCLCPP_WARN(get_logger(), "Goal rejected: %s", error.what()); }
    }

    void onOdometry(const nav_msgs::msg::Odometry &input)
    {
        try
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = input.header;
            pose.pose = input.pose.pose;
            if (pose.header.frame_id != frame_)
            {
                auto transform = tf_.lookupTransform(frame_, pose.header.frame_id, tf2::TimePointZero);
                tf2::doTransform(pose, pose, transform);
            }
            RobotState state;
            const auto &p = pose.pose.position;
            const auto &q = pose.pose.orientation;
            state.pose = {{p.x, p.y, p.z}, {q.w, q.x, q.y, q.z}};
            const auto &twist = input.twist.twist;
            state.twist = {{twist.linear.x, twist.linear.y, twist.linear.z},
                           {twist.angular.x, twist.angular.y, twist.angular.z}};
            state.stamp = rclcpp::Time(input.header.stamp).seconds();
            auto planar_pose = toPose2D(state.pose);
            if (!std::isfinite(planar_pose.x) || !std::isfinite(planar_pose.y) || !std::isfinite(planar_pose.yaw) ||
                !std::isfinite(state.twist.linear.x) || !std::isfinite(state.twist.angular.z)) return;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                double dt = state.stamp - state_.stamp;
                if (have_state_ && dt < -0.5) ++clock_epoch_;
                if (have_state_ && dt > 1e-3 && dt < 0.5)
                {
                    const auto &control = planner_->param();
                    state.acceleration.linear.x = std::clamp((state.twist.linear.x - state_.twist.linear.x) / dt,
                        -control.max_acceleration, control.max_acceleration);
                    state.acceleration.angular.z = std::clamp((state.twist.angular.z - state_.twist.angular.z) / dt,
                        -control.max_angular_acceleration, control.max_angular_acceleration);
                }
                // 差速车体的横向运动加速度，包含转弯的向心分量。
                state.acceleration.linear.y = state.twist.linear.x * state.twist.angular.z;
                state_ = state;
                have_state_ = true;
                odom_received_ = Steady::now();
                state_history_.push_back({state, ++state_id_});
                while (state_history_.size() > 400) state_history_.pop_front();
                if (replay::enabled()) replay::event({{"event", "pose"}, {"stamp", state.stamp},
                    {"clock_epoch", clock_epoch_}, {"state_id", state_id_}, {"state", replay::state(state)}});
            }
            processScan();
        }
        catch (const tf2::TransformException &) {}
    }

    // 仅做显示编码转换；规划仍共享感知生成的原始地图。
    nav_msgs::msg::OccupancyGrid mapMessage(const perception::OccupancyMap &map, rclcpp::Time stamp) const
    {
        Eigen::Vector3d lower, upper;
        map.bounds(lower, upper);
        nav_msgs::msg::OccupancyGrid message;
        message.header.frame_id = frame_;
        message.header.stamp = stamp;
        message.info.resolution = map.resolution();
        message.info.width = std::lround((upper.x() - lower.x()) / map.resolution());
        message.info.height = std::lround((upper.y() - lower.y()) / map.resolution());
        message.info.origin.position.x = lower.x();
        message.info.origin.position.y = lower.y();
        message.info.origin.orientation.w = 1;
        message.data.reserve(message.info.width * message.info.height);
        for (unsigned y = 0; y < message.info.height; ++y)
            for (unsigned x = 0; x < message.info.width; ++x)
            {
                auto cost = map.cost({lower.x() + (x + 0.5) * map.resolution(),
                                      lower.y() + (y + 0.5) * map.resolution(), 0});
                int value = cost == perception::Unknown ? -1 :
                            cost == perception::Lethal ? 100 :
                            cost == perception::Forbidden ? 99 :
                            cost == 0 ? 0 : 1 + (cost - 1) * 97 / 251;
                message.data.push_back(static_cast<std::int8_t>(value));
            }
        return message;
    }

    void processScan()
    {
        if (!pending_scan_ || state_history_.empty()) return;
        double stamp = rclcpp::Time(pending_scan_->header.stamp).seconds();
        double time = now().seconds();
        if (time < stamp) return;
        auto nearest = std::min_element(state_history_.begin(), state_history_.end(),
            [stamp](const RecordedState &a, const RecordedState &b)
            { return std::abs(a.state.stamp - stamp) < std::abs(b.state.stamp - stamp); });
        // 等待相邻的真值状态到达，保持真实时间戳，不用当前 pose 冒充扫描时刻的 pose。
        if (state_history_.back().state.stamp < stamp &&
            std::abs(nearest->state.stamp - stamp) > perception_->param().max_pose_scan_dt) return;
        auto message = std::move(pending_scan_);
        auto scan = std::make_shared<perception::LaserScan2D>();
        scan->stamp = stamp;
        scan->angle_min = message->angle_min;
        scan->angle_increment = message->angle_increment;
        scan->range_min = message->range_min;
        scan->range_max = message->range_max;
        scan->time_increment = message->time_increment;
        scan->ranges.assign(message->ranges.begin(), message->ranges.end());
        perception::PerceptionInput input;
        input.state = nearest->state;
        input.sensor = scan;
        input.now = time;
        input.robot = robot_;
        auto result = perception_->update(input);
        auto scan_id = ++scan_id_;
        if (!result.success)
        {
            replay::event({{"event", "perception"}, {"stamp", stamp}, {"clock_epoch", clock_epoch_},
                {"scan_id", scan_id}, {"state_id", nearest->id}, {"state", replay::state(nearest->state)}, {"success", false}});
            return;
        }
        if (replay::enabled()) replay::map({{"map_id", scan_id}, {"layer", "local"}, {"frame", frame_},
            {"stamp", result.stamp}, {"clock_epoch", clock_epoch_}, {"scan_id", scan_id},
            {"state_id", nearest->id}, {"state", replay::state(nearest->state)}, {"success", true}}, result.map);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            local_map_ = result.map;
            map_id_ = scan_id;
            map_received_ = Steady::now();
        }
        costmap_pub_->publish(mapMessage(*result.map,
            rclcpp::Time(static_cast<std::int64_t>(result.stamp * 1e9), RCL_ROS_TIME)));
    }

    void publishCommand()
    {
        geometry_msgs::msg::Twist command;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_goal_ && !publish_stop_) return;
            if (has_goal_ && command_version_ == goal_version_ && age(command_received_) <= command_timeout_ &&
                age(odom_received_) <= input_timeout_ && age(map_received_) <= map_timeout_)
                command = command_;
            publish_stop_ = false;
        }
        cmd_pub_->publish(command);
        replay::event({{"event", "command"}, {"stamp", now().seconds()}, {"clock_epoch", clock_epoch_},
                       {"command", {command.linear.x, command.angular.z}}});
    }

    bool current(std::uint64_t version)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_goal_ && goal_version_ == version && !stopping_;
    }

    void storeResult(const Snapshot &snapshot, planner::LocalPlanResult result, const char *status = nullptr)
    {
        if (age(snapshot.captured) > command_timeout_)
        {
            result.success = false;
            status = "planning_timeout";
            LOGW("ros2") << "planning_timeout age=" << age(snapshot.captured);
            planner_->discardWarmStart();
        }
        if (!status)
            status = result.goal_reached ? "goal_reached" : (result.success ? "planning_success" : "planning_failed");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_goal_ || goal_version_ != snapshot.version || stopping_)
            {
                recordPlan(snapshot, result, "discarded_goal_changed", false);
                return;
            }
            command_ = geometry_msgs::msg::Twist();
            if (result.success)
            {
                command_.linear.x = result.velocity;
                command_.angular.z = result.angular_velocity;
            }
            command_version_ = snapshot.version;
            command_received_ = Steady::now();
            if (result.goal_reached)
            {
                has_goal_ = false;
                publish_stop_ = true;
            }
        }
        recordPlan(snapshot, result, status, true);
        publishStatus(status);
        std::vector<Eigen::Vector3d> samples;
        if (result.success && result.trajectory.getPieceNum() > 0)
        {
            double duration = result.trajectory.getTotalDuration();
            for (double t = 0; t < duration; t += 0.05) samples.push_back(result.trajectory.getPos(t));
            samples.push_back(result.trajectory.getPos(duration));
            lookahead_pub_->publish(poseMessage(result.local_goal));
        }
        local_pub_->publish(pathMessage(samples));
        samples.clear();
        if (result.initial_trajectory.getPieceNum() > 0)
        {
            double duration = result.initial_trajectory.getTotalDuration();
            for (double t = 0; t < duration; t += 0.05) samples.push_back(result.initial_trajectory.getPos(t));
            samples.push_back(result.initial_trajectory.getPos(duration));
        }
        initial_pub_->publish(pathMessage(samples));
        LOGI("ros2") << "local " << status << " v=" << (result.success ? result.velocity : 0)
                     << " w=" << (result.success ? result.angular_velocity : 0);
    }

    void recordPlan(const Snapshot &snapshot, const planner::LocalPlanResult &result,
                    const char *status, bool accepted)
    {
        if (!replay::enabled()) return;
        replay::plan({{"stamp", std::isfinite(snapshot.state.stamp) ? snapshot.state.stamp : now().seconds()}, {"clock_epoch", snapshot.clock_epoch},
            {"plan_id", snapshot.plan_id}, {"state_id", snapshot.state_id}, {"map_id", snapshot.map_id},
            {"state", snapshot.state_id ? replay::state(snapshot.state) : nlohmann::json(nullptr)}, {"goal_id", snapshot.version},
            {"goal", {snapshot.goal.x(), snapshot.goal.y(), snapshot.goal.z()}},
            {"status", status}, {"accepted", accepted}, {"elapsed_ms", age(snapshot.captured) * 1000}},
            result, planner_->globalPath());
    }

    void planningLoop()
    {
        std::uint64_t planned_version = 0;
        MapPtr failed_global;
        while (true)
        {
            Snapshot snapshot;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait(lock, [this] { return stopping_ || requested_; });
                if (stopping_) return;
                requested_ = false;
                if (!has_goal_) continue;
                snapshot = {global_map_, local_map_, state_, goal_, goal_version_, Steady::now(), odom_received_, map_received_};
                snapshot.state_id = state_id_;
                snapshot.map_id = map_id_;
                snapshot.plan_id = ++plan_id_;
                snapshot.clock_epoch = clock_epoch_;
            }
            try
            {
                if (!snapshot.global || !snapshot.local || age(snapshot.odom_received) > input_timeout_ ||
                    age(snapshot.map_received) > map_timeout_)
                {
                    planner::LocalPlanResult result;
                    LOGW("ros2") << "waiting_for_fresh_inputs";
                    planner_->discardWarmStart();
                    storeResult(snapshot, result, "waiting_for_fresh_inputs");
                    continue;
                }
                if (planned_version != snapshot.version || planner_->globalPath().empty())
                {
                    if (planned_version == snapshot.version && failed_global == snapshot.global)
                    {
                        recordPlan(snapshot, {}, "global_search_failed_cached", false);
                        continue;
                    }
                    planner::GlobalPlanInput input;
                    input.map = snapshot.global;
                    input.start = toPose2D(snapshot.state.pose);
                    input.goal = {snapshot.goal.x(), snapshot.goal.y(), snapshot.goal.z()};
                    auto global = planner_->planGlobal(input);
                    planned_version = snapshot.version;
                    if (!current(snapshot.version))
                    {
                        recordPlan(snapshot, {}, "discarded_goal_changed", false);
                        continue;
                    }
                    global_pub_->publish(globalPathMessage(global.path));
                    if (!global.success)
                    {
                        failed_global = snapshot.global;
                        planner::LocalPlanResult result;
                        LOGW("ros2") << "global_search_failed";
                        storeResult(snapshot, result, "global_search_failed");
                        continue;
                    }
                    failed_global.reset();
                    LOGI("ros2") << "global success points=" << global.path.size();
                }
                planner::LocalPlanInput input;
                input.map = snapshot.local;
                input.state = snapshot.state;
                storeResult(snapshot, planner_->planLocal(input));
            }
            catch (const std::exception &error)
            {
                planner::LocalPlanResult result;
                LOGE("ros2") << "planning_error: " << error.what();
                storeResult(snapshot, result, "planning_error");
            }
        }
    }

    std::string frame_;
    double input_timeout_, map_timeout_, command_timeout_;
    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener listener_;
    std::shared_ptr<const robot_model::RobotModel> robot_;
    std::unique_ptr<planner::Planner> planner_;
    std::unique_ptr<perception::PerceptionInterface> perception_;
    std::deque<RecordedState> state_history_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr pending_scan_;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::thread worker_;
    bool stopping_ = false, requested_ = false, has_goal_ = false, have_state_ = false, publish_stop_ = false;
    std::uint64_t goal_version_ = 0, command_version_ = 0;
    std::uint64_t state_id_ = 0, scan_id_ = 0, map_id_ = 0, plan_id_ = 0, clock_epoch_ = 0;
    Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
    RobotState state_;
    MapPtr global_map_, local_map_;
    Steady::time_point odom_received_{}, map_received_{}, command_received_{};
    geometry_msgs::msg::Twist command_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_, costmap_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_pub_, local_pub_, initial_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_;
    rclcpp::TimerBase::SharedPtr plan_timer_, control_timer_, replay_timer_;
};
} // namespace navapex::ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<navapex::ros2::NavigationNode>()); }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("navapex"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
