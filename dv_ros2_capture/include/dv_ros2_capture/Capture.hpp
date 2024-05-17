#pragma once

// C++ System Headers
#include <iostream>
#include <vector>
#include <map>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <boost/thread/recursive_mutex.hpp>
#include <thread>
#include <boost/lockfree/spsc_queue.hpp>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/distortion_models.hpp"

#include <dv-processing/visualization/events_visualizer.hpp>
#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/core/core.hpp>
#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>
#include "dv_ros2_msgs/msg/event.hpp"
#include "dv_ros2_msgs/msg/event_array.hpp"
#include "dv_ros2_msgs/msg/trigger.hpp"
#include "dv_ros2_capture/Reader.hpp"
#include "dv_ros2_messaging/messaging.hpp"

namespace dv_ros2_capture
{
    using TimestampQueue = boost::lockfree::spsc_queue<int64_t, boost::lockfree::capacity<1000>>;
    struct Params
    {
        int64_t timeIncrement = 1000;
        bool frames           = true;
        bool events           = true;
        bool imu              = true;
        bool triggers         = true;
        std::string cameraName;
        std::filesystem::path aedat4FilePath;
        std::filesystem::path cameraCalibrationFilePath;
        std::string cameraFrameName    = "camera";
        std::string imuFrameName       = "imu";
        bool transformImuToCameraFrame = true;
        bool unbiasedImuData           = true;
        bool noiseFiltering            = false;
        int64_t noiseBATime            = 2000;

        std::vector<std::string> syncDeviceList;
        bool waitForSync = false;
        float rate = 1.0;
    };

    class Capture : public rclcpp::Node
    {
        using rclcpp::Node::Node;

    public:

        /// @brief Default constructor
        /// @param t_node_name name of the node
        Capture(const std::string &t_node_name);

        /// @brief Shallow copy constructor
        /// @param source object to copy
        Capture(const Capture &source);

        /// @brief Destructor
        ~Capture();

        /// @brief Start the threads for reading the data.
        void startCapture();

        /// @brief Stop the running threads.
        void stop();

        /// @brief Check if the threads are still running.
        /// @return true if the threads are running, false otherwise.
        bool isRunning() const;

    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_frame_publisher;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_camera_info_publisher;
        rclcpp::Publisher<dv_ros2_msgs::msg::EventArray>::SharedPtr m_events_publisher;
        rclcpp::Publisher<dv_ros2_msgs::msg::Trigger>::SharedPtr m_trigger_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher;
        
        std::unique_ptr<dv::noise::BackgroundActivityNoiseFilter<>> m_noise_filter = nullptr;

        /// Threads related
        std::thread m_frame_thread;
        TimestampQueue m_frame_queue;
        std::thread m_imu_thread;
        TimestampQueue m_imu_queue;
        std::thread m_events_thread;
        TimestampQueue m_events_queue;
        std::thread m_trigger_thread;
        TimestampQueue m_trigger_queue;
        std::atomic<bool> m_spin_thread = true;
        std::thread m_clock;
        std::thread m_sync_thread;
        std::unique_ptr<std::thread> m_discovery_thread = nullptr;
        std::unique_ptr<std::thread> m_camera_info_thread = nullptr;
        boost::recursive_mutex m_reader_mutex;
        std::atomic<bool> m_synchronized;
        std::atomic<int64_t> m_current_seek;

        /// @brief Parameter initialization
        inline void parameterInitilization() const;

        /// @brief Print parameters
        inline void parameterPrinter() const;

        /// @brief Reads the std library variables and ROS2 parameters
        /// @return true if all parameters are read successfully
        inline bool readParameters();

        /// @brief Populate the info message
        void populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry);

        /// @brief eclcpp node variable
        rclcpp::Node::SharedPtr m_node;

        /// @brief Timer for continous callback
        rclcpp::TimerBase::SharedPtr m_timer;

        /// @brief Parameters
        Params m_params;

        Reader m_reader;

        /// @brief Camera info msg
        sensor_msgs::msg::CameraInfo m_camera_info_msg;

        /// @brief reates and runs a thread that publishes discovery messages about the type of the camera.
        /// @param syncServiceName name of the service to be used for synchronization
        void runDiscovery(const std::string &syncServiceName); 

        /// @brief A blocking call that waits until all devices in 
        std::map<std::string, std::string> discoverSyncDevices() const;

        /// @brief Synchronization Thread
        void synchronizationThread();

        /// @brief Start a clock thread that gives time synchronization to all the other threads.
        /// @param start Start time of a recording file. (-1 if capturing from camera)
        /// @param end End time of a recording file. (-1 if capturing from camera)
        /// @param timeIncrement Increment of the timestamp at each iteration of the thread. The thread sleeps for.
        void clock(int64_t start, int64_t end, int64_t timeIncrement);

        void framePublisher();

        void imuPublisher();

        void eventsPublisher();

        void triggerPublisher();

    };
} // namespace dv_ros2_capture