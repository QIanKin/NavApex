#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <csignal>

namespace
{
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }

class PerceptionCostmap final : public nav2_costmap_2d::Costmap2DROS
{
public:
    PerceptionCostmap() : Costmap2DROS(rclcpp::NodeOptions()) {}

    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override
    {
        auto result = Costmap2DROS::on_activate(state);
        if (result == nav2_util::CallbackReturn::SUCCESS) createBond();
        return result;
    }
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
    {
        destroyBond();
        return Costmap2DROS::on_deactivate(state);
    }
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
    {
        // Humble 基类先销毁图层再停回调线程；这里先停线程，避免激光回调访问已销毁的图层。
        executor_thread_.reset();
        executor_.reset();
        auto result = Costmap2DROS::on_cleanup(state);
        callback_group_.reset();
        return result;
    }
};
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    // 使用 NodeOptions 构造，保留 launch 的节点名、命名空间和参数文件。
    auto costmap = std::make_shared<PerceptionCostmap>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(costmap->get_node_base_interface());
    while (rclcpp::ok() && !stopping) executor.spin_once(std::chrono::milliseconds(100));
    // ROS context 仍有效时先停地图更新线程，再清理插件，避免退出时并发销毁。
    if (costmap->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        costmap->deactivate();
    if (costmap->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        costmap->cleanup();
    executor.remove_node(costmap->get_node_base_interface());
    costmap.reset();
    rclcpp::shutdown();
    return 0;
}
