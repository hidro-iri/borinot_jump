#include "borinot_jump/jump_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<eagle_ros2_control::JumpController> controller =
        std::make_shared<eagle_ros2_control::JumpController>("jump_controller");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}