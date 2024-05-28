#include "dv_ros2_visualization/Visualizer.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string t_node_name("dv_ros2_visualization");

    std::shared_ptr<dv_ros2_visualization::Visualizer> vis_node = std::make_shared<dv_ros2_visualization::Visualizer>(t_node_name);

    auto handle = vis_node->add_on_set_parameters_callback(std::bind(&dv_ros2_visualization::Visualizer::paramsCallback, vis_node, std::placeholders::_1));

    vis_node->start();

    while (rclcpp::ok() && vis_node->isRunning())
    {
        rclcpp::spin_some(vis_node);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    rclcpp::shutdown();
    return 0;
}