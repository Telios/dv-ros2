#include "dv_ros2_accumulation/Accumulator.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string t_node_name{"dv_ros2_accumulation"};

    std::shared_ptr<dv_ros2_accumulation::Accumulator> accumulator = std::make_shared<dv_ros2_accumulation::Accumulator>(t_node_name);
    
    accumulator->start();

    while (rclcpp::ok() && accumulator->isRunning())
    {
        rclcpp::spin_some(accumulator);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    rclcpp::shutdown();
    return 0;
}