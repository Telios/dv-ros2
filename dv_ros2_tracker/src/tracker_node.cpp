#include "dv_ros2_tracker/Tracker.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string t_node_name{"dv_ros2_tracker"};

    std::shared_ptr<dv_ros2_tracker::Tracker> tracker = std::make_shared<dv_ros2_tracker::Tracker>(t_node_name);

    auto handle = tracker->add_on_set_parameters_callback(std::bind(&dv_ros2_tracker::Tracker::paramsCallback, tracker, std::placeholders::_1));

    // TODO: maybe have to start thread manually here

    rclcpp::spin(tracker);
    rclcpp::shutdown();
    return 0;
}