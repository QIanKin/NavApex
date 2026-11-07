#include <navapex_ros2/nav2_grid_map.hpp>
#include <planner/planner.hpp>
#include <tools/log.hpp>
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
using Costmap = nav2_msgs::msg::Costmap;

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
        logging.replay = false;
        planner_tools::initLog(logging);
        planner_ = std::make_unique<planner::Planner>("diff");
        if (std::abs(planner_->param().control_period * planning_frequency - 1) > 1e-6)
            throw std::invalid_argument("planning_frequency must match planner controller.period");
        auto global_topic = declare_parameter("global_costmap_topic", "/global_costmap/costmap_raw");
        auto local_topic = declare_parameter("local_costmap_topic", "/local_costmap/costmap_raw");
        auto map_qos = rclcpp::QoS(1).reliable().transient_local();
        global_sub_ = create_subscription<Costmap>(global_topic, map_qos, [this](Costmap::ConstSharedPtr message)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            global_map_ = std::move(message);
        });
        local_sub_ = create_subscription<Costmap>(local_topic, map_qos, [this](Costmap::ConstSharedPtr message)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            local_map_ = std::move(message);
            map_received_ = Steady::now();
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
        }
        plan_timer_ = create_wall_timer(std::chrono::duration<double>(1 / planning_frequency), [this]
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_goal_) return;
            requested_ = true;
            wake_.notify_one();
        });
        control_timer_ = create_wall_timer(std::chrono::duration<double>(1 / command_frequency), [this] { publishCommand(); });
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
        try { planner_tools::flushLog(); } catch (...) {}
    }

private:
    struct Snapshot
    {
        Costmap::ConstSharedPtr global, local;
        RobotState state;
        Eigen::Vector3d goal;
        std::uint64_t version;
        Steady::time_point captured, odom_received, map_received;
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
            std::lock_guard<std::mutex> lock(mutex_);
            double dt = state.stamp - state_.stamp;
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
        }
        catch (const tf2::TransformException &) {}
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
            if (!has_goal_ || goal_version_ != snapshot.version || stopping_) return;
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

    void planningLoop()
    {
        std::uint64_t planned_version = 0;
        Costmap::ConstSharedPtr failed_global;
        Costmap::ConstSharedPtr local_snapshot;
        std::shared_ptr<const Nav2GridMap> local_adapter;
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
                if (snapshot.global->header.frame_id != frame_ || snapshot.local->header.frame_id != frame_)
                    throw std::invalid_argument("Both costmaps must use the planning global_frame");
                if (planned_version != snapshot.version || planner_->globalPath().empty())
                {
                    if (planned_version == snapshot.version && failed_global == snapshot.global) continue;
                    planner::GlobalPlanInput input;
                    input.map = std::make_shared<Nav2GridMap>(snapshot.global);
                    input.start = toPose2D(snapshot.state.pose);
                    input.goal = {snapshot.goal.x(), snapshot.goal.y(), snapshot.goal.z()};
                    auto global = planner_->planGlobal(input);
                    planned_version = snapshot.version;
                    if (!current(snapshot.version)) continue;
                    global_pub_->publish(pathMessage(global.path));
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
                if (local_snapshot != snapshot.local)
                {
                    local_snapshot = snapshot.local;
                    local_adapter = std::make_shared<Nav2GridMap>(snapshot.local);
                }
                input.map = local_adapter;
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
    std::unique_ptr<planner::Planner> planner_;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::thread worker_;
    bool stopping_ = false, requested_ = false, has_goal_ = false, have_state_ = false, publish_stop_ = false;
    std::uint64_t goal_version_ = 0, command_version_ = 0;
    Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
    RobotState state_;
    Costmap::ConstSharedPtr global_map_, local_map_;
    Steady::time_point odom_received_{}, map_received_{}, command_received_{};
    geometry_msgs::msg::Twist command_;
    rclcpp::Subscription<Costmap>::SharedPtr global_sub_, local_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_pub_, local_pub_, initial_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_;
    rclcpp::TimerBase::SharedPtr plan_timer_, control_timer_;
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
