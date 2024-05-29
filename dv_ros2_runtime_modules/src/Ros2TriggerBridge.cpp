#include <dv_ros2_messaging/messaging.hpp>

#include <rclcpp/rclcpp.hpp>

#include <dv-sdk/module.hpp>

class Ros2TriggerBridge : public dv::ModuleBase
{
private:
    std::shared_ptr<rclcpp::Node> node = nullptr;
    std::shared_ptr<rclcpp::Publisher<dv_ros2_msgs::msg::Trigger>> trigger_pub = nullptr;

public:
    static void initInputs(dv::InputDefinitionList &in)
    {
        in.addTriggerInput("sync");
    }

    static const char *initDescription()
    {
        return ("Publishes trigger data as ROS2 publisher.");
    }

    static void initConfigOptions(dv::RuntimeConfig &config)
    {
        config.add("topicName", dv::ConfigOption::stringOption("ROS2 Topic name", "/camera/trigger"));
        config.setPriorityOptions({"topicName"});
    }

    void configUpdate() override
    {
        ModuleBase::configUpdate();
    }

    void run() override
    {
        if (!trigger_pub)
        {
            if (!rclcpp::ok())
            {
                char **argv = {nullptr};
                int argc = 0;
                rclcpp::init(argc, argv);
            }

            node = std::make_shared<rclcpp::Node>("DVS_Trigger_Publisher");

            std::string topicName = config.getString("topicName");

            trigger_pub = node->create_publisher<dv_ros2_msgs::msg::Trigger>(topicName, 10);
        }

        auto triggers = inputs.getTriggerInput("sync");
        if (auto input = triggers.data())
        {
            for (const auto &trigger : input)
            {
                trigger_pub->publish(dv_ros2_msgs::toRosTriggerMessage(trigger));
            }
        }
        rclcpp::spin_some(node);
    
    }
};

registerModuleClass(Ros2TriggerBridge);