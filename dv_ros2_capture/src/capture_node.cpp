#include "dv_ros2_capture/Capture.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string t_node_name{"dv_ros2_capture"};

    std::shared_ptr<dv_ros2_capture::Capture> capture = std::make_shared<dv_ros2_capture::Capture>(t_node_name);
    
    rclcpp::spin(capture);
    rclcpp::shutdown();
    return 0;
}