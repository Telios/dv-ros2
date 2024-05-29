#pragma once

#include <dv_ros2_messaging/messaging.hpp>

#include <dv_ros2_msgs/msg/event.hpp>
#include <dv_ros2_msgs/msg/event_array.hpp>

#include <boost/circular_buffer.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <dv-sdk/module.hpp>
#include <regex>
#include <sensor_msgs/msg/camera_info.hpp>

class Ros2EventBridge : public dv::ModuleBase
{
public:
	typedef dv::InputVectorDataWrapper<dv::EventPacket, dv::Event> DvEvents;
	typedef std::shared_ptr<DvEvents> DvEventsPtr;
	typedef boost::circular_buffer<DvEventsPtr> EventsBuffer;

private:
    static inline const std::regex filenameCleanupRegex{"[^a-zA-Z-_\\d]"};

    std::shared_ptr<rclcpp::Node> node = nullptr;
    std::shared_ptr<rclcpp::Publisher<dv_ros2_msgs::msg::EventArray>> event_pub = nullptr;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> camera_info_pub = nullptr;

    uint16_t width = 0;
    uint16_t height = 0;

    cv::Mat distortion_coeffs;
    cv::Mat camera_mat;

    EventsBuffer events_buffer;
    
    sensor_msgs::msg::CameraInfo camera_info;
    std::string camera_id;

    bool loadCalibrationFile(const std::string &filename);

    static bool cvExists(const cv::FileNode &fn);

    void setCameraID(const std::string &originDescription);

public:
    static void initInputs(dv::InputDefinitionList &in);
    
    static const char *initDescription();

    static void initConfigOptions(dv::RuntimeConfig &config);

    void run() override;

    void configUpdate() override;

    EventsBuffer::const_iterator findClosest(int64_t timestamp) const;

    void publishEventsMsg(const dv_ros2_msgs::msg::EventArray &msg);
};