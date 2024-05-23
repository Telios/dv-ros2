#include "dv-processing/processing.hpp"
#include "dv_ros2_messaging/messaging.hpp"

#include "rclcpp/rclcpp.hpp"
#include "thread"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string t_node_name("dv_ros2_visualization");

    std::shared_ptr<rclcpp::Node> vis_node = std::make_shared<rclcpp::Node>(t_node_name);

    dv::EventStreamSlicer slicer;
    std::unique_ptr<dv::visualization::EventVisualizer> visualizer = nullptr;

    auto eventSubscriber 
        = vis_node->create_subscription<dv_ros2_msgs::msg::EventArray>(
            "events", 200, 
            [&slicer, &visualizer, &vis_node](const dv_ros2_msgs::msg::EventArray::SharedPtr events)
            {
                if (visualizer == nullptr)
                {
                    visualizer = std::make_unique<dv::visualization::EventVisualizer>(cv::Size(events->width, events->height));
                }

                try
                {
                    slicer.accept(dv_ros2_msgs::toEventStore(*events));
                }
                catch(std::out_of_range &e)
                {
                    RCLCPP_WARN(vis_node->get_logger(), "%s", e.what());
                }
            }); 
    auto framePublisher = vis_node->create_publisher<sensor_msgs::msg::Image>("image", 10);

    slicer.doEveryTimeInterval(std::chrono::milliseconds(5), [&visualizer, &framePublisher](const dv::EventStore &events)
    {
        if (visualizer != nullptr)
        {
            cv::Mat image = visualizer->generateImage(events);
            sensor_msgs::msg::Image msg = dv_ros2_msgs::toRosImageMessage(image);
            msg.header.stamp = dv_ros2_msgs::toRosTime(events.getLowestTime());
            framePublisher->publish(msg);
        }
    });


    while(rclcpp::ok())
    {
        rclcpp::spin_some(vis_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}