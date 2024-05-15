#include "dv_ros2_accumulation/Accumulator.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string t_node_name{"dv_ros2_accumulation"};

    std::shared_ptr<dv_ros2_accumulation::Accumulator> accumulator = std::make_shared<dv_ros2_accumulation::Accumulator>(t_node_name);
    
    rclcpp::spin(accumulator);
    rclcpp::shutdown();
    return 0;
}