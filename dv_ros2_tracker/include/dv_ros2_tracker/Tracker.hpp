#pragma once

// C++ System Headers
#include <boost/lockfree/spsc_queue.hpp>
#include <thread>

// ROS2 Libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <dv_ros2_messaging/messaging.hpp>
#include <dv_ros2_msgs/msg/event_array.hpp>
#include <dv_ros2_msgs/msg/depth.hpp>
#include <dv_ros2_msgs/msg/timed_keypoint.hpp>
#include <dv_ros2_msgs/msg/timed_keypoint_array.hpp>
#include <sensor_msgs/distortion_models.hpp>

#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/core/core.hpp>
#include <dv-processing/features/event_combined_lk_tracker.hpp>
#include <dv-processing/features/feature_tracks.hpp>
#include <dv-processing/features/image_feature_lk_tracker.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/features/event_feature_lk_tracker.hpp>


namespace dv_ros2_tracker
{
    using TrackData = std::variant<dv::EventStore, dv_ros2_msgs::FrameMap, dv::kinematics::Transformationf>;
    using DataQueue = boost::lockfree::spsc_queue<TrackData, boost::lockfree::capacity<100>>;

    template<typename To, typename From>
    [[nodiscard]] inline std::unique_ptr<To> static_unique_ptr_cast(std::unique_ptr<From> &&ptr) {
        return std::unique_ptr<To>(static_cast<To *>(ptr.release()));
    }

    class Tracker : public rclcpp::Node
    {
        using rclcpp::Node::Node;
    public:
        /// @brief Default constructor
        /// @param t_node_name name of the node
        Tracker(const std::string &t_node_name);

        /// @brief Default destructor
        ~Tracker();

        /// @brief Start the tracker
        void startTracking();

        [[nodiscard]] bool isRunning() const;

        /// @brief Callback when parameters get changed.
        /// @param parameters Parameters
        /// @return SetParametersResult result of the callback
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter> &parameters);
    
    private:
        /// @brief Parameter initialization
        inline void parameterInitialization() const;

        /// @brief Print parameters
        inline void parameterPrinter() const;

        /// @brief Reads the std library variables and ROS2 parameters
        /// @return true if all parameters are read successfully
        inline bool readParameters();

        void updateConfiguration();

        void createTracker();

        enum class OperationMode
        {
            EventsOnly = 0,
            EventsOnlyCompensated,
            FramesOnly,
            FramesOnlyCompensated,
            Combined,
            CombinedCompensated
        };

        OperationMode mode = OperationMode::EventsOnly;

        struct Params
        {
            /// @brief Enable the use of event data, at least one event or frame input has to be enabled
            bool use_events = true;
            /// @brief Enable the use of frame data, at least one event or frame input has to be enabled
            bool use_frames = true;
            /// @brief Maximum number of features to track
            int32_t max_tracks = 300;
            /// @brief Number of pyramid layers to use for Lucas-Kanade tracking
            int32_t num_pyr_layers = 4;
            /// @brief Track termination epsilon for Lucas-Kanade tracking
            float termination_epsilon = 0.001;
            /// @brief Perform backward tracking and reject any tracks that don't "track-back" to original location
            bool lookback_rejection = false;
            /// @brief Search window size, this value is used for both x and y sizes
            int32_t search_window_size = 24;
            /// @brief Mask out regions of image where tracked features are present
            bool masked_feature_detect = true;
            /// @brief When tracked amount of features reached this threshold (proportion of maxTracks), new features will be detected
            float redetection_threshold = 0.75;
            /// @brief FAST corner detector threshold
            int32_t fast_threshold = 10;
            /// @brief Number of events accumulated in a single frame
            int32_t num_events = 30000;
            /// @brief Frame accumulation framerate
            int32_t accumulation_framerate = 50;
            /// @brief Combined mode uses accumulated frame to perform intermediate tracking between image frames, this value controls how many frames are accumulated between two image frames
            int32_t num_intermediate_frames = 5;
            /// @brief MotionAware Tracker
            bool use_motion_compensation = false;
        };

        dv::features::FeatureTracks frame_tracks;

        Params m_params;

        dv::features::ImageFeatureLKTracker::Config m_lucas_kanade_config;

        std::thread m_keypoints_thread;
        
        DataQueue m_data_queue;

        rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr m_events_array_subscriber;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_frame_subscriber;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_frame_info_subscriber;

        rclcpp::Subscription<dv_ros2_msgs::msg::Depth>::SharedPtr m_depth_estimation_subscriber;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_tf_subscriber;

        rclcpp::Publisher<dv_ros2_msgs::msg::TimedKeypointArray>::SharedPtr m_timed_keypoint_array_publisher;

        rclcpp::Publisher<dv_ros2_msgs::msg::TimedKeypointArray>::SharedPtr m_timed_keypoint_undistorted_array_publisher;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_tracks_preview_publisher;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_tracks_events_frames_publisher;

        rclcpp::Node::SharedPtr m_node;

        dv::features::TrackerBase::UniquePtr m_tracker = nullptr;

        void stop();

        void eventsArrayCallback(const dv_ros2_msgs::msg::EventArray::SharedPtr msgPtr);
        
        void frameCallback(const sensor_msgs::msg::Image::SharedPtr msgPtr);

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msgPtr);

        void depthEstimationCallback(const dv_ros2_msgs::msg::Depth::SharedPtr msgPtr);

        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msgPtr);

        float m_depth_estimation = 3.0;

        std::queue<dv::EventStore> m_queue_event_store;

        int64_t last_transform_time = 0;

        std::atomic<bool> m_spin_thread = false;

        dv::camera::calibrations::CameraCalibration m_camera_calibration;

        bool m_camera_initialized = false;

        dv::features::FeatureTracks m_frame_tracks;

        int64_t m_last_events_timestamp = 0;

        std::queue<dv_ros2_msgs::FrameMap> m_queue_frame;

        void assembleTrack();

        [[nodiscard]] inline dv_ros2_msgs::msg::TimedKeypointArray toRosTimedKeypointArrayMessage(
		const int64_t timestamp, const dv::cvector<dv::TimedKeyPoint> &keypoints) {
            dv_ros2_msgs::msg::TimedKeypointArray msg;

            msg.header.stamp = dv_ros2_msgs::toRosTime(timestamp);
            msg.keypoints.reserve(keypoints.size());

            for (const auto &kp : keypoints) {
                auto &k     = msg.keypoints.emplace_back();
                k.x         = kp.pt.x();
                k.y         = kp.pt.y();
                k.size      = kp.size;
                k.angle     = kp.angle;
                k.response  = kp.response;
                k.octave    = kp.octave;
                k.class_id  = kp.class_id;
                k.timestamp = kp.timestamp;
            }

            return msg;
	}

    void pushEventToTracker(const dv::EventStore &events);

    void pushFrameToTracker(const dv::Frame &frame);

    void pushTransformToTracker(const dv::kinematics::Transformationf &transform);

    dv::cvector<dv::TimedKeyPoint> undistortKeypoints(const dv::cvector<dv::TimedKeyPoint> &keypoints);

    bool runTracking();

    void publishPreview(const cv::Mat &background);

    void publishEventsPreview(const cv::Mat &background);

    void manageEventsQueue(const dv::EventStore &events);

    void manageFramesQueue(const dv_ros2_msgs::FrameMap &map);

    void manageTransformsQueue(const dv::kinematics::Transformationf &transform);
    };
}  // namespace dv_ros2_tracker