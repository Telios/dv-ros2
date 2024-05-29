#include "dv_ros2_tracker/Tracker.hpp"

namespace dv_ros2_tracker
{
    Tracker::Tracker(const std::string &t_node_name)
        : Node(t_node_name), m_node{this}
    {
        RCLCPP_INFO(m_node->get_logger(), "Constructor is initialized");
        parameterInitialization();
        if (!readParameters())
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameters");
            rclcpp::shutdown();
            std::exit(EXIT_FAILURE);
        }

        parameterPrinter();

        // Publishers
        m_timed_keypoint_array_publisher = m_node->create_publisher<dv_ros2_msgs::msg::TimedKeypointArray>("keypoints", 100);
        m_timed_keypoint_undistorted_array_publisher = m_node->create_publisher<dv_ros2_msgs::msg::TimedKeypointArray>("keypoints_undistorted", 100);

        // update configuration
        updateConfiguration();

        m_frame_info_subscriber = m_node->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10, std::bind(&Tracker::cameraInfoCallback, this, std::placeholders::_1));

        m_frame_tracks.setTrackTimeout(std::chrono::milliseconds(10));
    }

    Tracker::~Tracker()
    {
        stop();
    }

    void Tracker::eventsArrayCallback(const dv_ros2_msgs::msg::EventArray::SharedPtr msgPtr)
    {
        if (msgPtr == nullptr)
        {
            return;
        }
        auto events = dv_ros2_msgs::toEventStore(*msgPtr);
        m_data_queue.push(std::move(events));
    }

    void Tracker::frameCallback(const sensor_msgs::msg::Image::SharedPtr msgPtr)
    {
        if (msgPtr == nullptr)
        {
            return;
        }
        m_data_queue.push(dv_ros2_msgs::FrameMap(msgPtr));
    }

    void Tracker::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msgPtr)
    {
        // if the tracker is not initialized don't care about the messages.
        if (msgPtr == nullptr) 
        {
            return;
        }

        // if the timestamp is not monotonically increasing return.
        auto timestamp = dv_ros2_msgs::toDvTime(msgPtr->header.stamp);
        if (last_transform_time >= timestamp) 
        {
            return;
        }
        last_transform_time = timestamp;

        Eigen::Vector3f t(msgPtr->pose.position.x, msgPtr->pose.position.y, msgPtr->pose.position.z);

        Eigen::Quaternionf q(
            msgPtr->pose.orientation.w, msgPtr->pose.orientation.x, msgPtr->pose.orientation.y, msgPtr->pose.orientation.z);

        dv::kinematics::Transformationf T_WC = dv::kinematics::Transformationf(timestamp, t, q);
        m_data_queue.push(T_WC);
    }

    void Tracker::depthEstimationCallback(const dv_ros2_msgs::msg::Depth::SharedPtr msgPtr)
    {
        if (msgPtr == nullptr)
        {
            return;
        }
        m_depth_estimation = msgPtr->depth;
        auto timestamp = dv_ros2_msgs::toDvTime(msgPtr->timestamp);
        switch (mode)
        {
            case OperationMode::EventsOnlyCompensated:
                dynamic_cast<dv::features::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>> *>(m_tracker.get())
                    ->accept(dv::measurements::Depth(timestamp, m_depth_estimation));
                break;
            case OperationMode::FramesOnlyCompensated:
                dynamic_cast<dv::features::ImageFeatureLKTracker *>(m_tracker.get())
                    ->accept(dv::measurements::Depth(timestamp, m_depth_estimation));
                break;
            case OperationMode::CombinedCompensated:
                dynamic_cast<dv::features::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(m_tracker.get())
                    ->accept(dv::measurements::Depth(timestamp, m_depth_estimation));
                break;
            default:
                break;
        }
    }

    void Tracker::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msgPtr)
    {
        if (msgPtr == nullptr || m_camera_initialized)
        {
            return;
        }
        // read the camera info
        m_camera_calibration = dv::camera::calibrations::CameraCalibration();
        m_camera_calibration.resolution = cv::Size(static_cast<int>(msgPtr->width), static_cast<int>(msgPtr->height));
        for (const auto &d : msgPtr->d)
        {
            m_camera_calibration.distortion.push_back(static_cast<float>(d));
        }
        if (static_cast<std::string>(msgPtr->distortion_model) == sensor_msgs::distortion_models::EQUIDISTANT) {
		m_camera_calibration.distortionModel = dv::camera::DistortionModel::Equidistant;
        }
        else if (static_cast<std::string>(msgPtr->distortion_model) == sensor_msgs::distortion_models::PLUMB_BOB) {
            m_camera_calibration.distortionModel = dv::camera::DistortionModel::RadTan;
        }
        else {
            throw dv::exceptions::InvalidArgument<std::string>(
                "Unknown camera model.", msgPtr->distortion_model);
        }
        m_camera_calibration.focalLength    = cv::Point2f(static_cast<float>(msgPtr->k[0]), static_cast<float>(msgPtr->k[4]));
        m_camera_calibration.principalPoint = cv::Point2f(static_cast<float>(msgPtr->k[2]), static_cast<float>(msgPtr->k[5]));
        m_camera_initialized                = true;

        // create the tracker according to the config file and the camera info.
        createTracker();

        // Subscribers
        if (mode == OperationMode::FramesOnly || mode == OperationMode::Combined
            || mode == OperationMode::FramesOnlyCompensated || mode == OperationMode::CombinedCompensated) {
            m_frame_subscriber = m_node->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&Tracker::frameCallback, this, std::placeholders::_1));
            RCLCPP_INFO(m_node->get_logger(), "Subscribing to image stream..");
        }
        if (mode == OperationMode::EventsOnly || mode == OperationMode::Combined
            || mode == OperationMode::EventsOnlyCompensated || mode == OperationMode::CombinedCompensated) {
            m_events_array_subscriber = m_node->create_subscription<dv_ros2_msgs::msg::EventArray>("events", 10, std::bind(&Tracker::eventsArrayCallback, this, std::placeholders::_1));
            RCLCPP_INFO(m_node->get_logger(), "Subscribing to event stream..");
        }
        if (mode == OperationMode::FramesOnlyCompensated || mode == OperationMode::EventsOnlyCompensated
            || mode == OperationMode::CombinedCompensated) {
            RCLCPP_INFO(m_node->get_logger(), "Subscribing pose and depth messages..");
            m_depth_estimation_subscriber
                = m_node->create_subscription<dv_ros2_msgs::msg::Depth>("depth_estimation", 10, std::bind(&Tracker::depthEstimationCallback, this, std::placeholders::_1));
            m_tf_subscriber = m_node->create_subscription<geometry_msgs::msg::PoseStamped>("pose", 10, std::bind(&Tracker::poseCallback, this, std::placeholders::_1));
            RCLCPP_INFO(m_node->get_logger(), "Tracker with motion compensation..");
        }

        startTracking();
    }

    bool Tracker::isRunning() const
    {
        return m_spin_thread.load(std::memory_order_relaxed);
    }

    void Tracker::startTracking()
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning tracking node...");
        m_spin_thread = true;
        m_keypoints_thread = std::thread(&Tracker::assembleTrack, this);
    }

    void Tracker::createTracker()
    {
        auto detector = std::make_unique<dv::features::ImagePyrFeatureDetector>(m_camera_calibration.resolution, cv::FastFeatureDetector::create(m_params.fast_threshold));

        switch (mode)
        {
            case OperationMode::EventsOnly:
            {
                RCLCPP_INFO(m_node->get_logger(), "Constructing Events Only Tracker..");
                auto eventTracker = dv::features::EventFeatureLKTracker<dv::EdgeMapAccumulator>::RegularTracker(
                    m_camera_calibration.resolution, m_lucas_kanade_config, nullptr, std::move(detector),
                    std::make_unique<dv::features::FeatureCountRedetection>(m_params.redetection_threshold));
                eventTracker->setMaxTracks(m_params.max_tracks);
                eventTracker->setFramerate(m_params.accumulation_framerate);
                eventTracker->setNumberOfEvents(m_params.num_events);
                eventTracker->setLookbackRejection(m_params.lookback_rejection);
                m_tracker = static_unique_ptr_cast<dv::features::TrackerBase>(std::move(eventTracker));
                break;
            }
            case OperationMode::EventsOnlyCompensated: 
            {
                RCLCPP_INFO(m_node->get_logger(),"Constructing Events with Motion Compensation Tracker..");
                auto eventTracker = dv::features::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>>::MotionAwareTracker(
                    std::make_shared<dv::camera::CameraGeometry>(m_camera_calibration.getCameraGeometry()),
                    m_lucas_kanade_config, nullptr, nullptr, std::move(detector),
                    std::make_unique<dv::features::FeatureCountRedetection>(m_params.redetection_threshold));

                eventTracker->setMaxTracks(m_params.max_tracks);
                eventTracker->setFramerate(m_params.accumulation_framerate);
                eventTracker->setNumberOfEvents(m_params.num_events);
                eventTracker->setLookbackRejection(m_params.lookback_rejection);
                m_tracker = static_unique_ptr_cast<dv::features::TrackerBase>(std::move(eventTracker));
                break;
            }
            case OperationMode::FramesOnly: 
            {
                RCLCPP_INFO(m_node->get_logger(),"Constructing Frames Only Tracker..");
                auto frameTracker = dv::features::ImageFeatureLKTracker::RegularTracker(m_camera_calibration.resolution,
                    m_lucas_kanade_config, std::move(detector),
                    std::make_unique<dv::features::FeatureCountRedetection>(m_params.redetection_threshold));
                frameTracker->setMaxTracks(m_params.max_tracks);
                frameTracker->setLookbackRejection(m_params.lookback_rejection);
                m_tracker = static_unique_ptr_cast<dv::features::TrackerBase>(std::move(frameTracker));
                break;
            }
            case OperationMode::FramesOnlyCompensated: 
            {
                RCLCPP_INFO(m_node->get_logger(),"Constructing Frames with Motion Compensation Tracker..");
                auto frameTracker = dv::features::ImageFeatureLKTracker::MotionAwareTracker(
                    std::make_shared<dv::camera::CameraGeometry>(m_camera_calibration.getCameraGeometry()),
                    m_lucas_kanade_config, nullptr, std::move(detector),
                    std::make_unique<dv::features::FeatureCountRedetection>(m_params.redetection_threshold));
                frameTracker->setMaxTracks(m_params.max_tracks);
                frameTracker->setLookbackRejection(m_params.lookback_rejection);
                m_tracker = static_unique_ptr_cast<dv::features::TrackerBase>(std::move(frameTracker));
                break;
            }

            case OperationMode::Combined: 
            {
                RCLCPP_INFO(m_node->get_logger(),"Constructing Combined Tracker..");
                auto combinedTracker = dv::features::EventCombinedLKTracker<dv::EdgeMapAccumulator>::RegularTracker(
                    m_camera_calibration.resolution, m_lucas_kanade_config, nullptr, std::move(detector),
                    std::make_unique<dv::features::FeatureCountRedetection>(m_params.redetection_threshold));
                combinedTracker->setMaxTracks(m_params.max_tracks);
                combinedTracker->setNumberOfEvents(m_params.num_events);
                combinedTracker->setNumIntermediateFrames(m_params.num_intermediate_frames);
                combinedTracker->setLookbackRejection(m_params.lookback_rejection);
                m_tracker = static_unique_ptr_cast<dv::features::TrackerBase>(std::move(combinedTracker));
                break;
            }
            case OperationMode::CombinedCompensated: 
            {
                RCLCPP_INFO(m_node->get_logger(),"Constructing Combined with Motion Compensation Tracker..");
                auto combinedTracker = dv::features::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>>::MotionAwareTracker(
                    std::make_shared<dv::camera::CameraGeometry>(m_camera_calibration.getCameraGeometry()),
                    m_lucas_kanade_config, nullptr, nullptr, std::move(detector),
                    std::make_unique<dv::features::FeatureCountRedetection>(m_params.redetection_threshold));
                combinedTracker->setMaxTracks(m_params.max_tracks);
                combinedTracker->setNumberOfEvents(m_params.num_events);
                combinedTracker->setNumIntermediateFrames(m_params.num_intermediate_frames);
                combinedTracker->setLookbackRejection(m_params.lookback_rejection);
                m_tracker = static_unique_ptr_cast<dv::features::TrackerBase>(std::move(combinedTracker));
                break;
            }
        }
    }

    void Tracker::pushEventToTracker(const dv::EventStore &events)
    {
        switch (mode)
        {
            case OperationMode::EventsOnly:
			    dynamic_cast<dv::features::EventFeatureLKTracker<dv::EdgeMapAccumulator> *>(m_tracker.get())->accept(events);
			    break;
            case OperationMode::EventsOnlyCompensated:
                dynamic_cast<dv::features::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>> *>(m_tracker.get())
                    ->accept(events);
                break;
            case OperationMode::Combined:
                dynamic_cast<dv::features::EventCombinedLKTracker<dv::EdgeMapAccumulator> *>(m_tracker.get())->accept(events);
                break;
            case OperationMode::CombinedCompensated:
                dynamic_cast<dv::features::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(m_tracker.get())
                    ->accept(events);
                break;
            default:
                // Noop,
                break;
        }
    }

    void Tracker::pushFrameToTracker(const dv::Frame &frame)
    {
        switch (mode)
        {
            case OperationMode::FramesOnly:
                dynamic_cast<dv::features::ImageFeatureLKTracker *>(m_tracker.get())->accept(frame);
                break;
            case OperationMode::FramesOnlyCompensated:
                dynamic_cast<dv::features::ImageFeatureLKTracker *>(m_tracker.get())->accept(frame);
                break;
            case OperationMode::Combined:
                dynamic_cast<dv::features::EventCombinedLKTracker<dv::EdgeMapAccumulator> *>(m_tracker.get())->accept(frame);
                break;
            case OperationMode::CombinedCompensated:
                dynamic_cast<dv::features::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(m_tracker.get())
                    ->accept(frame);
                break;
            default:
                // Noop
                break;

        }
    }

    void Tracker::pushTransformToTracker(const dv::kinematics::Transformationf &transform)
    {
        switch (mode)
        {
            case OperationMode::EventsOnlyCompensated:
                dynamic_cast<dv::features::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>> *>(m_tracker.get())
                    ->accept(transform);
                break;
            case OperationMode::FramesOnlyCompensated:
                dynamic_cast<dv::features::ImageFeatureLKTracker *>(m_tracker.get())->accept(transform);
                break;
            case OperationMode::CombinedCompensated:
                dynamic_cast<dv::features::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(m_tracker.get())
                    ->accept(transform);
                break;
            default:
                break;
        }
    }

    dv::cvector<dv::TimedKeyPoint> Tracker::undistortKeypoints(const dv::cvector<dv::TimedKeyPoint> &keypoints)
    {
        static const auto mCamGeom                          = m_camera_calibration.getCameraGeometry();
        dv::cvector<dv::TimedKeyPoint> undistortedKeypoints = keypoints;
        for (auto &keypoint : undistortedKeypoints) 
        {
            keypoint.pt = mCamGeom.undistort<dv::Point2f>(keypoint.pt);
        }
        return undistortedKeypoints;
    }

    bool Tracker::runTracking()
    {
        if (auto tracks = m_tracker->runTracking(); tracks != nullptr)
        {
            frame_tracks.accept(tracks);
            m_timed_keypoint_array_publisher->publish(toRosTimedKeypointArrayMessage(tracks->timestamp, tracks->keypoints));

            m_timed_keypoint_undistorted_array_publisher->publish(toRosTimedKeypointArrayMessage(tracks->timestamp, undistortKeypoints(tracks->keypoints)));
            return true;
        }
        return false;
    }

    void Tracker::publishEventsPreview(const cv::Mat &background)
    {
        if (m_tracks_events_frames_publisher->get_subscription_count() > 0 && !background.empty())
        {
            m_tracks_events_frames_publisher->publish(dv_ros2_msgs::toRosImageMessage(frame_tracks.visualize(background)));
        }
    }

    void Tracker::publishPreview(const cv::Mat &background)
    {
        if (m_tracks_preview_publisher->get_subscription_count() > 0 && !background.empty())
        {
            m_tracks_preview_publisher->publish(dv_ros2_msgs::toRosImageMessage(frame_tracks.visualize(background)));
        }
    }

    void Tracker::manageEventsQueue(const dv::EventStore &events)
    {
        switch (mode)
        {
            case OperationMode::EventsOnly: 
            {
                pushEventToTracker(events);
                while (runTracking()) {
                    cv::Mat accumulatedImage
                        = dynamic_cast<dv::features::EventFeatureLKTracker<dv::EdgeMapAccumulator> *>(m_tracker.get())
                            ->getAccumulatedFrame();
                    // publish accumulated image
                    publishEventsPreview(accumulatedImage);
                }
                break;
		    }
            case OperationMode::Combined: 
            {
                pushEventToTracker(events);
                m_last_events_timestamp = events.getHighestTime();
                // synchronize frames and events
                while (!m_queue_frame.empty() && m_queue_frame.front().frame.timestamp < m_last_events_timestamp) {
                    pushFrameToTracker(m_queue_frame.front().frame);
                    runTracking();
                    auto frames = dynamic_cast<dv::features::EventCombinedLKTracker<dv::EdgeMapAccumulator> *>(m_tracker.get())
                                    ->getAccumulatedFrames();
                    if (!frames.empty()) {
                        cv::Mat accumulatedImage = frames.back().pyramid.front();
                        // publish accumulated image
                        publishEventsPreview(accumulatedImage);
                    }
                    // publish the tracks on the frame
                    publishPreview(m_queue_frame.front().frame.image);

                    // remove the used data from the queue
                    m_queue_frame.pop();
                }
                break;
            }
                // same behaviour for the two cases
            case OperationMode::EventsOnlyCompensated:
            case OperationMode::CombinedCompensated:
                // Store events batch in the queue for synchronization
                m_queue_event_store.push(events);
                break;
            default:
                break;

        }
    }

    void Tracker::manageFramesQueue(const dv_ros2_msgs::FrameMap &map)
    {
        if (mode == OperationMode::FramesOnly) 
        {
            // Perform tracking and publish the results
            pushFrameToTracker(map.frame);
            runTracking();
            publishPreview(map.frame.image);
        }
        else 
        {
            // store frame in the queue for synchronization
            m_queue_frame.push(map);
        }
    }

    void Tracker::manageTransformsQueue(const dv::kinematics::Transformationf &transform)
    {
        pushTransformToTracker(transform);
        switch (mode)
        {
            case OperationMode::FramesOnlyCompensated: 
            {
                // Synchronize frames and transforms
                while (!m_queue_frame.empty() && m_queue_frame.front().frame.timestamp < last_transform_time) 
                {
                    pushFrameToTracker(m_queue_frame.front().frame);
                    runTracking();
                    publishPreview(m_queue_frame.front().frame.image);

                    m_queue_frame.pop();
                }
                break;
		    }
            case OperationMode::EventsOnlyCompensated: 
            {
                // Synchronize events and transforms
                while (!m_queue_event_store.empty() && m_queue_event_store.front().getHighestTime() < last_transform_time) 
                {
                    pushEventToTracker(m_queue_event_store.front());
                    m_last_events_timestamp = m_queue_event_store.front().getHighestTime();
                    m_queue_event_store.pop();
                    while (runTracking()) 
                    {
                        cv::Mat accumulatedImage
                            = dynamic_cast<dv::features::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>> *>(m_tracker.get())
                                ->getAccumulatedFrame();
                        publishEventsPreview(accumulatedImage);
                    }
                }
                break;
            }
            case OperationMode::CombinedCompensated: 
            {
                // Synchronize events and transforms
                while (!m_queue_event_store.empty() && m_queue_event_store.front().getHighestTime() < last_transform_time) 
                {
                    pushEventToTracker(m_queue_event_store.front());
                    m_last_events_timestamp = m_queue_event_store.front().getHighestTime();
                    m_queue_event_store.pop();
                    // Synchronize frames and transforms
                    while (!m_queue_frame.empty() && m_queue_frame.front().frame.timestamp < m_last_events_timestamp) 
                    {
                        pushFrameToTracker(m_queue_frame.front().frame);
                        // perform tracking and publish the results
                        runTracking();
                        auto tmpTracker = dynamic_cast<dv::features::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(
                            m_tracker.get());
                        auto frames = tmpTracker->getAccumulatedFrames();
                        if (!frames.empty()) 
                        {
                            cv::Mat accumulatedImage = frames.back().pyramid.front();
                            publishEventsPreview(accumulatedImage);
                        }
                        publishPreview(m_queue_frame.front().frame.image);

                        m_queue_frame.pop();
                    }
                }
                break;
            }
            default:
                break;
        }
    }

    void Tracker::assembleTrack()
    {
        while (m_spin_thread)
        {
            m_data_queue.consume_all([&](const auto &data)
            {
                // read events.
                if (const dv::EventStore *events = std::get_if<dv::EventStore>(&data); events != nullptr) 
                {
                    manageEventsQueue(*events);
                }
                // read frames
                else if (const dv_ros2_msgs::FrameMap *map = std::get_if<dv_ros2_msgs::FrameMap>(&data); map != nullptr) 
                {
                    manageFramesQueue(*map);
                }
                // read transform
                else if (const dv::kinematics::Transformationf *transform
                        = std::get_if<dv::kinematics::Transformationf>(&data);
                        transform != nullptr) 
                {
                    manageTransformsQueue(*transform);
                }
                else 
                {
                    throw std::runtime_error("Wrong type in queue.");
                }
            });
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    void Tracker::stop()
    {
        RCLCPP_INFO(m_node->get_logger(), "Stopping the tracking node...");
        m_spin_thread = false;
        m_keypoints_thread.join();
    }
       

    inline void Tracker::parameterInitialization() const
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange int_range;
        rcl_interfaces::msg::FloatingPointRange float_range;
        descriptor.set__description("Enable the use of event data, at least one event or frame input has to be enabled");
        m_node->declare_parameter("use_events", m_params.use_events, descriptor);
        descriptor.set__description("Enable the use of frame data, at least one event or frame input has to be enabled");
        m_node->declare_parameter("use_frames", m_params.use_frames, descriptor);
        descriptor.set__description("Maximum number of features to track");
        int_range.set__from_value(10).set__to_value(1000);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("max_tracks", m_params.max_tracks, descriptor);
        descriptor.set__description("Number of pyramid layers to use for Lucas-Kanade tracking");
        int_range.set__from_value(1).set__to_value(10);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("num_pyr_layers", m_params.num_pyr_layers, descriptor);
        descriptor.set__description("Track termination epsilon for Lucas-Kanade tracking");
        float_range.set__from_value(0.0001).set__to_value(0.1);
        descriptor.floating_point_range = {float_range};
        m_node->declare_parameter("termination_epsilon", m_params.termination_epsilon, descriptor);
        descriptor.set__description("Perform backward tracking and reject any tracks that don't 'track-back' to original location");
        m_node->declare_parameter("lookback_rejection", m_params.lookback_rejection, descriptor);
        descriptor.set__description("Search window size, this value is used for both x and y sizes");
        int_range.set__from_value(10).set__to_value(100);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("search_window_size", m_params.search_window_size, descriptor);
        descriptor.set__description("Mask out regions of image where tracked features are present");
        m_node->declare_parameter("masked_feature_detect", m_params.masked_feature_detect, descriptor);
        descriptor.set__description("When tracked amount of features reached this threshold (proportion of maxTracks), new features will be detected");
        float_range.set__from_value(0.1).set__to_value(1.0);
        descriptor.floating_point_range = {float_range};
        m_node->declare_parameter("redetection_threshold", m_params.redetection_threshold, descriptor);
        descriptor.set__description("FAST corner detector threshold");
        int_range.set__from_value(1).set__to_value(100);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("fast_threshold", m_params.fast_threshold, descriptor);
        descriptor.set__description("Number of events accumulated in a single frame");
        int_range.set__from_value(1000).set__to_value(100000);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("num_events", m_params.num_events, descriptor);
        descriptor.set__description("Frame accumulation framerate");
        int_range.set__from_value(1).set__to_value(100);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("accumulation_framerate", m_params.accumulation_framerate, descriptor);
        descriptor.set__description("Combined mode uses accumulated frame to perform intermediate tracking between image frames, this value controls how many frames are accumulated between two image frames");
        int_range.set__from_value(1).set__to_value(10);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("num_intermediate_frames", m_params.num_intermediate_frames, descriptor);
        descriptor.set__description("MotionAware Tracker");
        m_node->declare_parameter("use_motion_compensation", m_params.use_motion_compensation, descriptor);
    }

    inline void Tracker::parameterPrinter() const
    {
        RCLCPP_INFO(m_node->get_logger(), "-------- Parameters --------");
        RCLCPP_INFO(m_node->get_logger(), "use_events: %s", m_params.use_events ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "use_frames: %s", m_params.use_frames ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "max_tracks: %d", m_params.max_tracks);
        RCLCPP_INFO(m_node->get_logger(), "num_pyr_layers: %d", m_params.num_pyr_layers);
        RCLCPP_INFO(m_node->get_logger(), "termination_epsilon: %f", m_params.termination_epsilon);
        RCLCPP_INFO(m_node->get_logger(), "lookback_rejection: %s", m_params.lookback_rejection ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "search_window_size: %d", m_params.search_window_size);
        RCLCPP_INFO(m_node->get_logger(), "masked_feature_detect: %s", m_params.masked_feature_detect ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "redetection_threshold: %f", m_params.redetection_threshold);
        RCLCPP_INFO(m_node->get_logger(), "fast_threshold: %d", m_params.fast_threshold);
        RCLCPP_INFO(m_node->get_logger(), "num_events: %d", m_params.num_events);
        RCLCPP_INFO(m_node->get_logger(), "accumulation_framerate: %d", m_params.accumulation_framerate);
        RCLCPP_INFO(m_node->get_logger(), "num_intermediate_frames: %d", m_params.num_intermediate_frames);
        RCLCPP_INFO(m_node->get_logger(), "use_motion_compensation: %s", m_params.use_motion_compensation ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "-----------------------------");
    }

    inline bool Tracker::readParameters()
    {
        if (!m_node->get_parameter("use_events", m_params.use_events))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter use_events");
            return false;
        }
        if (!m_node->get_parameter("use_frames", m_params.use_frames))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter use_frames");
            return false;
        }
        if (!m_node->get_parameter("max_tracks", m_params.max_tracks))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter max_tracks");
            return false;
        }
        if (!m_node->get_parameter("num_pyr_layers", m_params.num_pyr_layers))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter num_pyr_layers");
            return false;
        }
        if (!m_node->get_parameter("termination_epsilon", m_params.termination_epsilon))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter termination_epsilon");
            return false;
        }
        if (!m_node->get_parameter("lookback_rejection", m_params.lookback_rejection))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter lookback_rejection");
            return false;
        }
        if (!m_node->get_parameter("search_window_size", m_params.search_window_size))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter search_window_size");
            return false;
        }
        if (!m_node->get_parameter("masked_feature_detect", m_params.masked_feature_detect))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter masked_feature_detect");
            return false;
        }
        if (!m_node->get_parameter("redetection_threshold", m_params.redetection_threshold))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter redetection_threshold");
            return false;
        }
        if (!m_node->get_parameter("fast_threshold", m_params.fast_threshold))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter fast_threshold");
            return false;
        }
        if (!m_node->get_parameter("num_events", m_params.num_events))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter num_events");
            return false;
        }
        if (!m_node->get_parameter("accumulation_framerate", m_params.accumulation_framerate))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter accumulation_framerate");
            return false;
        }
        if (!m_node->get_parameter("num_intermediate_frames", m_params.num_intermediate_frames))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter num_intermediate_frames");
            return false;
        }
        if (!m_node->get_parameter("use_motion_compensation", m_params.use_motion_compensation))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter use_motion_compensation");
            return false;
        }
        return true;
    }

    void Tracker::updateConfiguration()
    {
        // Define the operation mode
        if (m_params.use_events && m_params.use_frames)
        {
            mode = OperationMode::Combined;
            m_tracks_preview_publisher = m_node->create_publisher<sensor_msgs::msg::Image>("preview/image", 10);
            m_tracks_events_frames_publisher = m_node->create_publisher<sensor_msgs::msg::Image>("events_preview/image", 10);
        }
        else if (m_params.use_events)
        {
            mode = OperationMode::EventsOnly;
            m_tracks_events_frames_publisher = m_node->create_publisher<sensor_msgs::msg::Image>("events_preview/image", 10);
        }
        else if (m_params.use_frames)
        {
            mode = OperationMode::FramesOnly;
            m_tracks_preview_publisher = m_node->create_publisher<sensor_msgs::msg::Image>("preview/image", 10);
        }
        else
        {
            throw dv::exceptions::RuntimeError("Neither events nor frames are enabled as input, at least one has to be enabled for the tracker!");
        }

        if (m_params.use_motion_compensation)
        {
            mode = static_cast<OperationMode>(static_cast<int>(mode) + 1);
        }

        m_lucas_kanade_config.maskedFeatureDetect = m_params.masked_feature_detect;
        m_lucas_kanade_config.numPyrLayers = m_params.num_pyr_layers;
        m_lucas_kanade_config.searchWindowSize = cv::Size(m_params.search_window_size, m_params.search_window_size);
        m_lucas_kanade_config.terminationEpsilon = m_params.termination_epsilon;
        
        createTracker();
    }

    rcl_interfaces::msg::SetParametersResult Tracker::paramsCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "use_events")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    m_params.use_events = param.as_bool();
                }
                else
                {
                    result.successful = false;
                    result.reason = "use_events parameter must be a boolean";
                }
            }
            else if (param.get_name() == "use_frames")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    m_params.use_frames = param.as_bool();
                }
                else
                {
                    result.successful = false;
                    result.reason = "use_frames parameter must be a boolean";
                }
            }
            else if (param.get_name() == "max_tracks")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.max_tracks = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "max_tracks parameter must be an integer";
                }
            }
            else if (param.get_name() == "num_pyr_layers")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.num_pyr_layers = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "num_pyr_layers parameter must be an integer";
                }
            }
            else if (param.get_name() == "termination_epsilon")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    m_params.termination_epsilon = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "termination_epsilon parameter must be a double";
                }
            }
            else if (param.get_name() == "lookback_rejection")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    m_params.lookback_rejection = param.as_bool();
                }
                else
                {
                    result.successful = false;
                    result.reason = "lookback_rejection parameter must be a boolean";
                }
            }
            else if (param.get_name() == "search_window_size")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.search_window_size = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "search_window_size parameter must be an integer";
                }
            }
            else if (param.get_name() == "masked_feature_detect")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    m_params.masked_feature_detect = param.as_bool();
                }
                else
                {
                    result.successful = false;
                    result.reason = "masked_feature_detect parameter must be a boolean";
                }
            }
            else if (param.get_name() == "redetection_threshold")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    m_params.redetection_threshold = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "redetection_threshold parameter must be a double";
                }
            }
            else if (param.get_name() == "fast_threshold")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.fast_threshold = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "fast_threshold parameter must be an integer";
                }
            }
            else if (param.get_name() == "num_events")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.num_events = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "num_events parameter must be an integer";
                }
            }
            else if (param.get_name() == "accumulation_framerate")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.accumulation_framerate = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "accumulation_framerate parameter must be an integer";
                }
            }
            else if (param.get_name() == "num_intermediate_frames")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.num_intermediate_frames = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "num_intermediate_frames parameter must be an integer";
                }
            }
            else if (param.get_name() == "use_motion_compensation")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    m_params.use_motion_compensation = param.as_bool();
                }
                else
                {
                    result.successful = false;
                    result.reason = "use_motion_compensation parameter must be a boolean";
                }
            }
            else
            {
                result.successful = false;
                result.reason = "Unknown parameter";
            }
        }
        updateConfiguration();
        return result;
    }
} // namespace dv_ros2_tracker

