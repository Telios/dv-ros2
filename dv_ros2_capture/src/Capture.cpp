#include "dv_ros2_capture/Capture.hpp"

namespace dv_ros2_capture
{
    Capture::Capture(const std::string &t_node_name) 
    : Node(t_node_name), m_node{this}
    {
        m_spin_thread = true;
        RCLCPP_INFO(m_node->get_logger(), "Constructor is initialized");
        parameterInitilization();

        if (!readParameters())
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameters");
            rclcpp::shutdown();
            std::exit(EXIT_FAILURE);
        }

        parameterPrinter();

        if (m_params.aedat4FilePath.empty())
        {
            m_reader = Reader(m_params.cameraName);
        }
        else 
        {
            m_reader = Reader(m_params.aedat4FilePath, m_params.cameraName);
        }
        // startupTime = rclcpp::Time::now();

        if (m_params.frames && !m_reader.isFrameStreamAvailable())
        {
            m_params.frames = false;
            RCLCPP_WARN(m_node->get_logger(), "Frame stream is not available.");
        }
        if (m_params.events && !m_reader.isEventStreamAvailable())
        {
            m_params.events = false;
            RCLCPP_WARN(m_node->get_logger(), "Event stream is not available.");
        }
        if (m_params.imu && !m_reader.isImuStreamAvailable())
        {
            m_params.imu = false;
            RCLCPP_WARN(m_node->get_logger(), "IMU stream is not available.");
        }
        if (m_params.triggers && !m_reader.isTriggerStreamAvailable())
        {
            m_params.triggers = false;
            RCLCPP_WARN(m_node->get_logger(), "Trigger stream is not available.");
        }

        if (m_params.frames)
        {
            m_frame_publisher = m_node->create_publisher<sensor_msgs::msg::Image>("camera/frame", 10);
        }
        if (m_params.events)
        {
            m_events_publisher = m_node->create_publisher<dv_ros2_msgs::msg::EventArray>("camera/events", 10);
        }
        if (m_params.triggers)
        {
            m_trigger_publisher = m_node->create_publisher<dv_ros2_msgs::msg::Trigger>("camera/trigger", 10);
        }
        if (m_params.imu)
        {
            m_imu_publisher = m_node->create_publisher<sensor_msgs::msg::Imu>("camera/imu", 10);
        }
        m_camera_info_publisher = m_node->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);


        // TODO camera calibration

        std::optional<cv::Size> resolution;
        if (m_reader.isFrameStreamAvailable())
        {
            resolution = m_reader.getFrameResolution();
        }
        else if (m_reader.isEventStreamAvailable())
        {
            resolution = m_reader.getEventResolution();
        }
        if (resolution.has_value())
        {
            const auto width = static_cast<float>(resolution->width);
            populateInfoMsg(dv::camera::CameraGeometry(width, width, width * 0.5f, static_cast<float>(resolution->height) * 0.5f, *resolution));
            // TODO: genrate calibration file
        }
        else
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to get camera resolution");
            rclcpp::shutdown();
            std::exit(EXIT_FAILURE);
        }

        auto &camera_ptr = m_reader.getCameraCapturePtr();
        if (camera_ptr != nullptr) {
            if (camera_ptr->isFrameStreamAvailable()) 
            {
                // DAVIS camera

                // TODO: dynamic reconfigure

                if (camera_ptr->isTriggerStreamAvailable()) {
                    // External trigger detection support for DAVIS346 - MODIFY HERE FOR DIFFERENT DETECTION SETTINGS!
                    camera_ptr->deviceConfigSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES, true);
                    camera_ptr->deviceConfigSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES, false);
                    camera_ptr->deviceConfigSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES, false);
                    camera_ptr->deviceConfigSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, m_params.triggers);
                }
            }
            else {
                // DVXplorer type camera
                
                // TODO: dynamic reconfigure

                if (camera_ptr->isTriggerStreamAvailable()) {
                    // External trigger detection support for DVXplorer - MODIFY HERE FOR DIFFERENT DETECTION SETTINGS!
                    camera_ptr->deviceConfigSet(DVX_EXTINPUT, DVX_EXTINPUT_DETECT_RISING_EDGES, true);
                    camera_ptr->deviceConfigSet(DVX_EXTINPUT, DVX_EXTINPUT_DETECT_FALLING_EDGES, false);
                    camera_ptr->deviceConfigSet(DVX_EXTINPUT, DVX_EXTINPUT_DETECT_PULSES, false);
                    camera_ptr->deviceConfigSet(DVX_EXTINPUT, DVX_EXTINPUT_RUN_DETECTOR, m_params.triggers);
                }
            }

            // Support variable data interval sizes.
            camera_ptr->deviceConfigSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, m_params.timeIncrement);
        }
        else {
            // TODO: dynamic reconfigure
        }

        RCLCPP_INFO(m_node->get_logger(), "Successfully launched.");
    }

    Capture::~Capture()
    {
        RCLCPP_INFO(m_node->get_logger(), "Destructor is initialized");
        stop();
        rclcpp::shutdown();
    }

    void Capture::stop()
    {
        RCLCPP_INFO(m_node->get_logger(), "Stopping the capture node...");
        m_spin_thread = false;
        m_clock.join();
        if (m_params.frames)
        {
            m_frame_thread.join();
        }
        if (m_params.events)
        {
            m_events_thread.join();
        }
        if (m_params.triggers)
        {
            m_trigger_thread.join();
        }
        if (m_params.imu)
        {
            m_imu_thread.join();
        }
        if (m_sync_thread.joinable())
        {
            m_sync_thread.join();
        }
        if (m_camera_info_thread != nullptr)
        {
            m_camera_info_thread->join();
        }
        if (m_discovery_thread != nullptr)
        {
            m_discovery_thread->join();
        }
    }

    inline void Capture::parameterInitilization() const
    {
        m_node->declare_parameter("time_increment", m_params.timeIncrement);
        m_node->declare_parameter("frames", m_params.frames);
        m_node->declare_parameter("events", m_params.events);
        m_node->declare_parameter("imu", m_params.imu);
        m_node->declare_parameter("triggers", m_params.triggers);
        m_node->declare_parameter("camera_name", m_params.cameraName);
        m_node->declare_parameter("aedat4_file_path", m_params.aedat4FilePath);
        m_node->declare_parameter("camera_calibration_file_path", m_params.cameraCalibrationFilePath);
        m_node->declare_parameter("camera_frame_name", m_params.cameraFrameName);
        m_node->declare_parameter("imu_frame_name", m_params.imuFrameName);
        m_node->declare_parameter("transform_imu_to_camera_frame", m_params.transformImuToCameraFrame);
        m_node->declare_parameter("unbiased_imu_data", m_params.unbiasedImuData);
        m_node->declare_parameter("noise_filtering", m_params.noiseFiltering);
        m_node->declare_parameter("noise_ba_time", m_params.noiseBATime);
        m_node->declare_parameter("sync_device_list", m_params.syncDeviceList);
        m_node->declare_parameter("wait_for_sync", m_params.waitForSync);
        m_node->declare_parameter("rate", m_params.rate);
    }

    inline void Capture::parameterPrinter() const
    {
        RCLCPP_INFO(m_node->get_logger(), "---- Parameters ----");
        RCLCPP_INFO(m_node->get_logger(), "time_increment: %d", m_params.timeIncrement);
        RCLCPP_INFO(m_node->get_logger(), "frames: %s", m_params.frames ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "events: %s", m_params.events ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "imu: %s", m_params.imu ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "triggers: %s", m_params.triggers ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "camera_name: %s", m_params.cameraName.c_str());
        RCLCPP_INFO(m_node->get_logger(), "aedat4_file_path: %s", m_params.aedat4FilePath.c_str());
        RCLCPP_INFO(m_node->get_logger(), "camera_calibration_file_path: %s", m_params.cameraCalibrationFilePath.c_str());
        RCLCPP_INFO(m_node->get_logger(), "camera_frame_name: %s", m_params.cameraFrameName.c_str());
        RCLCPP_INFO(m_node->get_logger(), "imu_frame_name: %s", m_params.imuFrameName.c_str());
        RCLCPP_INFO(m_node->get_logger(), "transform_imu_to_camera_frame: %s", m_params.transformImuToCameraFrame ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "unbiased_imu_data: %s", m_params.unbiasedImuData ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "noise_filtering: %s", m_params.noiseFiltering ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "noise_ba_time: %d", m_params.noiseBATime);
        RCLCPP_INFO(m_node->get_logger(), "sync_device_list: ");
        for (const auto &device : m_params.syncDeviceList)
        {
            RCLCPP_INFO(m_node->get_logger(), "  %s", device.c_str());
        }
        RCLCPP_INFO(m_node->get_logger(), "wait_for_sync: %s", m_params.waitForSync ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "rate: %f", m_params.rate);
    }

    inline bool Capture::readParameters()
    {
        if (!m_node->get_parameter("time_increment", m_params.timeIncrement))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter time_increment");
            return false;
        }
        if (!m_node->get_parameter("frames", m_params.frames))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter frames");
            return false;
        }
        if (!m_node->get_parameter("events", m_params.events))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter events");
            return false;
        }
        if (!m_node->get_parameter("imu", m_params.imu))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter imu");
            return false;
        }
        if (!m_node->get_parameter("triggers", m_params.triggers))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter triggers");
            return false;
        }
        if (!m_node->get_parameter("camera_name", m_params.cameraName))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter camera_name");
            return false;
        }
        if (!m_node->get_parameter("aedat4_file_path", m_params.aedat4FilePath))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter aedat4_file_path");
            return false;
        }
        if (!m_node->get_parameter("camera_calibration_file_path", m_params.cameraCalibrationFilePath))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter camera_calibration_file_path");
            return false;
        }
        if (!m_node->get_parameter("camera_frame_name", m_params.cameraFrameName))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter camera_frame_name");
            return false;
        }
        if (!m_node->get_parameter("imu_frame_name", m_params.imuFrameName))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter imu_frame_name");
            return false;
        }
        if (!m_node->get_parameter("transform_imu_to_camera_frame", m_params.transformImuToCameraFrame))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter transform_imu_to_camera_frame");
            return false;
        }
        if (!m_node->get_parameter("unbiased_imu_data", m_params.unbiasedImuData))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter unbiased_imu_data");
            return false;
        }
        if (!m_node->get_parameter("noise_filtering", m_params.noiseFiltering))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter noise_filtering");
            return false;
        }
        if (!m_node->get_parameter("noise_ba_time", m_params.noiseBATime))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter noise_ba_time");
            return false;
        }
        if (!m_node->get_parameter("sync_device_list", m_params.syncDeviceList))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter sync_device_list");
            return false;
        }
        if (!m_node->get_parameter("wait_for_sync", m_params.waitForSync))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter wait_for_sync");
            return false;
        }
        if (!m_node->get_parameter("rate", m_params.rate))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter rate");
            return false;
        }
        return true;
    }

    void Capture::populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry)
    {
        m_camera_info_msg.width = cameraGeometry.getResolution().width;
        m_camera_info_msg.height = cameraGeometry.getResolution().height;

        const auto distortion = cameraGeometry.getDistortion();

        switch (cameraGeometry.getDistortionModel()) 
        {
            case dv::camera::DistortionModel::Equidistant: {
                m_camera_info_msg.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
                m_camera_info_msg.d.assign(distortion.begin(), distortion.end());
                break;
            }

            case dv::camera::DistortionModel::RadTan: {
                m_camera_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
                m_camera_info_msg.d.assign(distortion.begin(), distortion.end());
                if (m_camera_info_msg.d.size() < 5) {
                    m_camera_info_msg.d.resize(5, 0.0);
                }
                break;
            }

            case dv::camera::DistortionModel::None: {
                m_camera_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
                m_camera_info_msg.d                = {0.0, 0.0, 0.0, 0.0, 0.0};
                break;
            }

            default:
                throw dv::exceptions::InvalidArgument<dv::camera::DistortionModel>(
                    "Unsupported camera distortion model.", cameraGeometry.getDistortionModel());
	}

        auto cx = cameraGeometry.getCentralPoint().x;
        auto cy = cameraGeometry.getCentralPoint().y;
        auto fx = cameraGeometry.getFocalLength().x;
        auto fy = cameraGeometry.getFocalLength().y;

        m_camera_info_msg.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        m_camera_info_msg.r = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
        m_camera_info_msg.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0};



    }

    void Capture::startCapture()
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning capture node...");
        auto times = m_reader.getTimeRange();

        const auto &live_capture = m_reader.getCameraCapturePtr();

        if (!live_capture)  //TODO: alter to live_capture
        {
            m_synchronized = false;
            m_sync_thread = std::thread(&Capture::synchronizationThread, this);
        }
        else 
        {
            m_synchronized = true;
        }

        if (times.has_value())
        {
            m_clock = std::thread(&Capture::clock, this, times->first, times->second, m_params.timeIncrement);
        }
        else
        {
            m_clock = std::thread(&Capture::clock, this, -1, -1, m_params.timeIncrement);
        }
        if (m_params.frames)
        {
            m_frame_thread = std::thread(&Capture::framePublisher, this);
        }
        if (m_params.events)
        {
            m_events_thread = std::thread(&Capture::eventsPublisher, this);
        }
        if (m_params.triggers)
        {
            m_trigger_thread = std::thread(&Capture::triggerPublisher, this);
        }
        if (m_params.imu)
        {
            m_imu_thread = std::thread(&Capture::imuPublisher, this);
        }

        if (m_params.events || m_params.frames) 
        {
            m_camera_info_thread = std::make_unique<std::thread>([this] 
            {
                rclcpp::Rate infoRate(25.0);
                while (m_spin_thread.load(std::memory_order_relaxed))
                {
                    // TODO: publish camera info
                }
                infoRate.sleep();

            });

        }



    }

    bool Capture::isRunning() const
    {
        return m_spin_thread.load(std::memory_order_relaxed);
    }

    void Capture::synchronizationThread()
    {
        /*
        std::string serviceName;
        const auto &liveCapture = m_reader.getCameraCapturePtr();
        if (liveCapture->isMasterCamera()) 
        {
            // Wait for all cameras to show up
            const auto syncServiceList = discoverSyncDevices();
            runDiscovery(serviceName);
            sendSyncCalls(syncServiceList);
            mSynchronized = true;
        }
        else 
        {
            mSyncServerService = std::make_unique<ros::ServiceServer>(mNodeHandle->advertiseService(
                fmt::format("{}/sync", liveCapture->getCameraName()), &Capture::synchronizeCamera, this));

            serviceName = mSyncServerService->getService();
            runDiscovery(serviceName);

            // Wait for synchronization only if explicitly requested
            if (!mParams.waitForSync) 
            {
                mSynchronized = true;
            }

            size_t iterations = 0;
            while (mSpinThread.load(std::memory_order_relaxed)) {
                std::this_thread::sleep_for(1ms);

                // Do not print warnings if it's synchronized
                if (mSynchronized.load(std::memory_order_relaxed)) {
                    continue;
                }

                if (iterations > 2000) {
                    ROS_WARN_STREAM("[" << liveCapture->getCameraName() << "] Waiting for synchronization service call...");
                    iterations = 0;
                }
                iterations++;
            }
        }
        */

    }

    void Capture::clock(int64_t start, int64_t end, int64_t timeIncrement)
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning clock.");

        double frequency = 1.0 / (static_cast<double>(timeIncrement) * 1e-6);
        
        rclcpp::Rate sleepRate(frequency);
        if (start == -1)
        {
            start = std::numeric_limits<int64_t>::max() - 1;
            end = std::numeric_limits<int64_t>::max();
            timeIncrement = 0;
            RCLCPP_INFO_STREAM(m_node->get_logger(), "Reading from camera [" << m_reader.getCameraName() << "]");
        }

        while (m_spin_thread)
        {
            if (m_synchronized.load(std::memory_order_relaxed))
            {
                if (m_params.frames)
                {
                    m_frame_queue.push(start);
                }
                if (m_params.events)
                {
                    m_events_queue.push(start);
                }
                if (m_params.triggers)
                {
                    m_trigger_queue.push(start);
                }
                if (m_params.imu)
                {
                    m_imu_queue.push(start);
                }
                start += timeIncrement;
            }

            sleepRate.sleep();

            if (start >= end || !m_reader.isConnected())
            {
                m_spin_thread = false;
            }
        }

    }

    void Capture::framePublisher()
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning frame publisher.");
        
        std::optional<dv::Frame> frame = std::nullopt;

        while (m_spin_thread)
        {
            m_frame_queue.consume_all([&](const int64_t timestamp)
            {
                if (!frame.has_value())
                {
                    std::lock_guard<boost::recursive_mutex> lockGuard(m_reader_mutex);
                    frame = m_reader.getNextFrame();
                }
                while (frame.has_value() && timestamp >= frame->timestamp)
                {
                    if (m_frame_publisher->get_subscription_count() > 0)
                    {
                        // TODO: publish frame from dv::Frame to ROS msg port dv-ros-messaging package
                    }
                }
            });
        }
    }

    void Capture::imuPublisher()
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning imu publisher.");
    }

    void Capture::eventsPublisher()
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning events publisher.");
        
        std::optional<dv::EventStore> events = std::nullopt;

        cv::Size resolution = m_reader.getEventResolution().value();

        while (m_spin_thread)
        {
            m_events_queue.consume_all([&](const int64_t timestamp)
            {
                if (!events.has_value())
                {
                    std::lock_guard<boost::recursive_mutex> lockGuard(m_reader_mutex);
                    events = m_reader.getNextEventBatch();
                }
                while (events.has_value() && !events->isEmpty() && timestamp >= events->getHighestTime()) 
                {
				    dv::EventStore store;
                    if (m_noise_filter != nullptr) 
                    {
                        m_noise_filter->accept(*events);
                        store = m_noise_filter->generateEvents();
                    }
                    else 
                    {
                        store = *events;
                    }

                    if (m_events_publisher->get_subscription_count() > 0) 
                    {
                        // TODO: convert from dv::EventStore to ROS msg; port dv-ros-messaging package
                        auto msg = dv_ros2_msgs::msg::EventArray();
                        rclcpp::Time time = {static_cast<uint32_t>(store.getLowestTime() / 1'000'000), static_cast<uint32_t>(store.getLowestTime() % 1'000'000) * 1'000};

                        int64_t secInMicro = static_cast<int64_t>(time.seconds()) * 1'000'000;
                        msg.header.stamp = rclcpp::Time(static_cast<double>(store.getHighestTime()) * 1e-6);
                        msg.events.reserve(store.size());
                        for (const auto &event : store)
                        {
                            int64_t time_diff = event.timestamp() - secInMicro;
                            if (time_diff < 1'000'000)
                            {
                                // We are in the same second, we only need to update the nano-second part
                                time = {static_cast<uint32_t>(time.seconds()), static_cast<uint32_t>(time_diff * 1'000)};
                            }
                            else
                            {
                                time = {static_cast<uint32_t>(event.timestamp() / 1'000'000), static_cast<uint32_t>(event.timestamp() % 1'000'000) * 1'000};
                                secInMicro = static_cast<int64_t>(time.seconds()) * 1'000'000;
                            }
                            auto &e = msg.events.emplace_back();
                            e.x = event.x();
                            e.y = event.y();
                            e.polarity = event.polarity();
                            e.ts = time;
                        }
                        msg.width = resolution.width;
                        msg.height = resolution.height;
                        m_events_publisher->publish(msg);
                    }
                    m_current_seek = store.getHighestTime();

                    std::lock_guard<boost::recursive_mutex> lockGuard(m_reader_mutex);
                    events = m_reader.getNextEventBatch();
                }

                if (events.has_value() && events->isEmpty()) 
                {
                    events = std::nullopt;
                }
            });
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    void Capture::triggerPublisher()
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning trigger publisher.");
    }






}