#include "dv_ros2_capture/Capture.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string t_node_name{"dv_ros2_capture"};

    std::shared_ptr<dv_ros2_capture::Capture> capture = std::make_shared<dv_ros2_capture::Capture>(t_node_name);
    
    capture->startCapture();

    while (rclcpp::ok() && capture->isRunning())
    {
        rclcpp::spin_some(capture);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    rclcpp::shutdown();
    return 0;
}