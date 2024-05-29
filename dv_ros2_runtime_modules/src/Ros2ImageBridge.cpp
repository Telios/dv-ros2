#include <dv_ros2_messaging/messaging.hpp>

#include <rclcpp/rclcpp.hpp>

#include <dv-sdk/module.hpp>

class Ros2ImageBridge : public dv::ModuleBase
{
private:
    std::shared_ptr<rclcpp::Node> node = nullptr;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub = nullptr;

public:
    static void initInputs(dv::InputDefinitionList &in)
    {
        in.addFrameInput("image");
    }

    static const char *initDescription()
    {
        return ("Publishes image frames into ROS2.");
    }
    
    static void initConfigOptions(dv::RuntimeConfig &config)
    {
        config.add("topicName", dv::ConfigOption::stringOption("ROS2 Topic name", "/camera/image"));
        config.setPriorityOptions({"topicName"});
    }

    void configUpdate() override
    {
        ModuleBase::configUpdate();
    }

    void run() override
    {
        if (!image_pub)
        {
            if (!rclcpp::ok())
            {
                char **argv = {nullptr};
                int argc = 0;
                rclcpp::init(argc, argv);
            }

            node = std::make_shared<rclcpp::Node>("DVS_Image_Publisher");

            std::string topicName = config.getString("topicName");

            image_pub = node->create_publisher<sensor_msgs::msg::Image>(topicName, 10);
        }

        auto frame = inputs.getFrameInput("image");
        image_pub->publish(dv_ros2_msgs::frameToRosImageMessage(*frame.frame().getBasePointer()));
        rclcpp::spin_some(node);
    }
};

registerModuleClass(Ros2ImageBridge);