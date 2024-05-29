#include <dv_ros2_messaging/messaging.hpp>

#include <rclcpp/rclcpp.hpp>

#include <dv-sdk/module.hpp>

class Ros2IMUBridge : public dv::ModuleBase
{
private:
    std::shared_ptr<rclcpp::Node> node = nullptr;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub = nullptr;

public:
    static void initInputs(dv::InputDefinitionList &in)
    {
        in.addIMUInput("imu");
    }

    static const char *initDescription()
    {
        return ("Publishes IMU measurements into ROS2.");
    }

    static void initConfigOptions(dv::RuntimeConfig &config)
    {
        config.add("topicName", dv::ConfigOption::stringOption("ROS2 Topic name", "/camera/imu"));
        config.setPriorityOptions({"topicName"});
    }

    void configUpdate() override
    {
        ModuleBase::configUpdate();
    }

    void run() override
    {
        if (!imu_pub)
        {
            if (!rclcpp::ok())
            {
                char **argv = {nullptr};
                int argc = 0;
                rclcpp::init(argc, argv);
            }

            node = std::make_shared<rclcpp::Node>("DVS_IMU_Publisher");

            std::string topicName = config.getString("topicName");

            imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(topicName, 10);
        }

        auto imuInput = inputs.getIMUInput("imu");
        for (const auto &sample : imuInput.data())
        {
            imu_pub->publish(dv_ros2_msgs::toRosImuMessage(sample));
        }
        rclcpp::spin_some(node);
    }
};

registerModuleClass(Ros2IMUBridge);