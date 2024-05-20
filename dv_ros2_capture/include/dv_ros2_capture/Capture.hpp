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
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

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
#include "dv_ros2_msgs/srv/synchronize_camera.hpp"
#include "dv_ros2_msgs/srv/set_imu_info.hpp"
#include "dv_ros2_msgs/srv/set_imu_biases.hpp"
#include "dv_ros2_msgs/msg/camera_discovery.hpp"
#include "dv_ros2_msgs/msg/imu_info.hpp"

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
        rclcpp::Publisher<dv_ros2_msgs::msg::CameraDiscovery>::SharedPtr m_discovery_publisher;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_transform_publisher;
        
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

        dv::camera::CalibrationSet m_calibration;

        int64_t m_imu_time_offset = 0;
        Eigen::Vector3f m_acc_biases = Eigen::Vector3f::Zero();
        Eigen::Vector3f m_gyro_biases = Eigen::Vector3f::Zero();
        rclcpp::Time startup_time;
        std::optional<tf2_msgs::msg::TFMessage> m_imu_to_cam_transforms = std::nullopt;
        dv::kinematics::Transformationf m_imu_to_cam_transform =
            dv::kinematics::Transformationf(0, Eigen::Vector3f::Zero(), Eigen::Quaternion<float>::Identity());

        /// @brief Parameter initialization
        inline void parameterInitilization() const;

        /// @brief Print parameters
        inline void parameterPrinter() const;

        /// @brief Reads the std library variables and ROS2 parameters
        /// @return true if all parameters are read successfully
        inline bool readParameters();

        /// @brief Populate the info message
        void populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry);

        /// @brief Convert the imu message frame into the camera frame if the transformation exists.
        /// @param imu
        /// @return ROS2 Imu message in camera reference frame
        [[nodiscard]] inline sensor_msgs::msg::Imu transformImuFrame(sensor_msgs::msg::Imu &&imu);


        /// @brief Service to set the IMU biases
        /// @param request_header Request header.
        /// @param req       Synchronization request.
        /// @param rsp       Synchronization response.
        /// @return true if the service call is successful
        bool setCameraInfo(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
                               std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> rsp);
        
        /// @brief Service to set the IMU info
        /// @param request_header Request header.
        /// @param req       Synchronization request.
        /// @param rsp       Synchronization response.
        /// @return true if the service call is successful
        bool setImuInfo(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<dv_ros2_msgs::srv::SetImuInfo::Request> req,
                               std::shared_ptr<dv_ros2_msgs::srv::SetImuInfo::Response> rsp);

        /// @brief Service to set the Camera info
        /// @param request_header Request header.
        /// @param req       Synchronization request.
        /// @param rsp       Synchronization response.
        /// @return true if the service call is successful
        bool setImuBiases(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<dv_ros2_msgs::srv::SetImuBiases::Request> req,
                               std::shared_ptr<dv_ros2_msgs::srv::SetImuBiases::Response> rsp);

        /// @brief Generate the CalibrationSet with the data from the Set Camera Info and the set IMU services.
        void updateCalibrationSet();

        /// @brief Stores the calibration data into a new file.
        /// @return Path to the new file.
        [[nodiscard]] fs::path saveCalibration();

        /// @brief Write current capture node calibration parameters into an active calibration file.
        void generateActiveCalibrationFile();

        /// @brief Get the path to the active calibration file.
        /// @return Filesystem path to the currently opened camera active calibration file.
        [[nodiscard]] fs::path getActiveCalibrationPath() const;

        /// @brief Get camera calibration directory for the currently opened camera, it uses
        /// @param createDirectories If true, the method will create the directory if it's not existing in the filesystem.
        /// @return Path to the calibration
        fs::path getCameraCalibrationDirectory(bool createDirectories = true) const;

        /// Handler for the camera synchronization service.
        /// @param request_header Request header.
        /// @param req       Synchronization request.
        /// @param rsp       Synchronization response.
        void synchronizeCamera(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<dv_ros2_msgs::srv::SynchronizeCamera::Request> req,
                               std::shared_ptr<dv_ros2_msgs::srv::SynchronizeCamera::Response> rsp);

        
        /// @brief A blocking call that waits until all devices in `mParams.syncDeviceList` is online and discovered over
        ///        the /dvs/discovery topic.
        /// @return A map of devices, where key is the camera name and value is the name of synchronization service.
        [[nodiscard]] std::map<std::string, std::string> discoverSyncDevices() const;

        /// @brief Send synchronization calls to the list of devices. This function distributes the timestamp offset to the
        ///        given devices over synchronization service calls. The list of devices should be retrieved by
        ///        `discoverSyncDevices` call.
        /// @param serviceNames  List of devices to synchronize.
        /// @sa Capture::discoverSyncDevices
        void sendSyncCalls(const std::map<std::string, std::string> &serviceNames) const;

        /// @brief rclcpp node variable
        rclcpp::Node::SharedPtr m_node;

        /// @brief Timer for continous callback
        rclcpp::TimerBase::SharedPtr m_timer;

        /// @brief Parameters
        Params m_params;

        /// @brief Reader object
        Reader m_reader;

        /// @brief Camera info msg
        sensor_msgs::msg::CameraInfo m_camera_info_msg;

        /// @brief reates and runs a thread that publishes discovery messages about the type of the camera.
        /// @param syncServiceName name of the service to be used for synchronization
        void runDiscovery(const std::string &syncServiceName); 

        /// @brief Synchronization Thread
        void synchronizationThread();

        /// @brief Start a clock thread that gives time synchronization to all the other threads.
        /// @param start Start time of a recording file. (-1 if capturing from camera)
        /// @param end End time of a recording file. (-1 if capturing from camera)
        /// @param timeIncrement Increment of the timestamp at each iteration of the thread. The thread sleeps for.
        void clock(int64_t start, int64_t end, int64_t timeIncrement);

        /// @brief Multi-threaded function to read the frames generated by the event camera.
        void framePublisher();

        /// @brief Multi-threaded function to read the imu data generated by the event camera.
        void imuPublisher();

        /// @brief Multi-threaded function to read the events generated by the event camera.
        void eventsPublisher();

        /// @brief Multi-threaded function to read the triggers generated by the event camera.
        void triggerPublisher();
    };
} // namespace dv_ros2_capture