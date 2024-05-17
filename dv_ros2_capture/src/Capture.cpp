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
        startup_time = m_node->now();

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
        RCLCPP_INFO(m_node->get_logger(), "time_increment: %d", static_cast<int>(m_params.timeIncrement));
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
        RCLCPP_INFO(m_node->get_logger(), "noise_ba_time: %d", static_cast<int>(m_params.noiseBATime));
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

    sensor_msgs::msg::Imu Capture::transformImuFrame(sensor_msgs::msg::Imu &&imu)
    {
        if (m_params.unbiasedImuData)
        {
            imu.linear_acceleration.x -= m_acc_biases.x();
            imu.linear_acceleration.y -= m_acc_biases.y();
            imu.linear_acceleration.z -= m_acc_biases.z();

            imu.angular_velocity.x -= m_gyro_biases.x();
            imu.angular_velocity.y -= m_gyro_biases.y();
            imu.angular_velocity.z -= m_gyro_biases.z();
        }
        if (m_params.transformImuToCameraFrame)
        {
            const Eigen::Vector3<double> resW 
                = m_imu_to_cam_transform.rotatePoint<Eigen::Vector3<double>>(imu.angular_velocity);
            imu.angular_velocity.x = resW.x();
            imu.angular_velocity.y = resW.y();
            imu.angular_velocity.z = resW.z();

            const Eigen::Vector3<double> resV
                = m_imu_to_cam_transform.rotatePoint<Eigen::Vector3<double>>(imu.linear_acceleration);
            imu.linear_acceleration.x = resV.x();
            imu.linear_acceleration.y = resV.y();
            imu.linear_acceleration.z = resV.z();
        }
        return imu;
    }
    
    void Capture::updateCalibrationSet()
    {
        RCLCPP_INFO(m_node->get_logger(), "Updating calibration set...");
        const std::string cameraName = m_reader.getCameraName();
        dv::camera::calibrations::CameraCalibration calib;
        bool calibrationExists = false;
        if (auto camCalibration = m_calibration.getCameraCalibrationByName(cameraName); camCalibration.has_value()) 
        {
            calib             = *camCalibration;
            calibrationExists = true;
        }
        else 
        {
            calib.name = cameraName;
        }
        calib.resolution = cv::Size(static_cast<int>(m_camera_info_msg.width), static_cast<int>(m_camera_info_msg.height));
        calib.distortion.clear();
        calib.distortion.assign(m_camera_info_msg.d.begin(), m_camera_info_msg.d.end());
        if (static_cast<std::string>(m_camera_info_msg.distortion_model) == sensor_msgs::distortion_models::PLUMB_BOB) 
        {
            calib.distortionModel = dv::camera::DistortionModel::RadTan;
        }
        else if (static_cast<std::string>(m_camera_info_msg.distortion_model) == sensor_msgs::distortion_models::EQUIDISTANT) 
        {
            calib.distortionModel = dv::camera::DistortionModel::Equidistant;
        }
        else 
        {
            throw dv::exceptions::InvalidArgument<sensor_msgs::msg::CameraInfo::_distortion_model_type>(
                "Unknown camera model.", m_camera_info_msg.distortion_model);
        }
        calib.focalLength = cv::Point2f(static_cast<float>(m_camera_info_msg.k[0]), static_cast<float>(m_camera_info_msg.k[4]));
        calib.principalPoint
            = cv::Point2f(static_cast<float>(m_camera_info_msg.k[2]), static_cast<float>(m_camera_info_msg.k[5]));

        calib.transformationToC0 = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};

        if (calibrationExists) 
        {
            m_calibration.updateCameraCalibration(calib);
        }
        else 
        {
            m_calibration.addCameraCalibration(calib);
        }

        dv::camera::calibrations::IMUCalibration imuCalibration;
        bool imuCalibrationExists = false;
        if (auto imuCalib = m_calibration.getImuCalibrationByName(cameraName); imuCalib.has_value()) 
        {
            imuCalibration       = *imuCalib;
            imuCalibrationExists = true;
        }
        else 
        {
            imuCalibration.name = cameraName;
        }
        bool imuHasValues = false;
        if ((m_imu_to_cam_transforms.has_value() && !m_imu_to_cam_transforms->transforms.empty())) 
        {
            const Eigen::Matrix4f mat         = m_imu_to_cam_transform.getTransform().transpose();
            imuCalibration.transformationToC0 = std::vector<float>(mat.data(), mat.data() + mat.rows() * mat.cols());
            imuHasValues                      = true;
        }

        if (!m_acc_biases.isZero()) 
        {
            imuCalibration.accOffsetAvg.x = m_acc_biases.x();
            imuCalibration.accOffsetAvg.y = m_acc_biases.y();
            imuCalibration.accOffsetAvg.z = m_acc_biases.z();
            imuHasValues                  = true;
        }

        if (!m_gyro_biases.isZero()) 
        {
            imuCalibration.omegaOffsetAvg.x = m_gyro_biases.x();
            imuCalibration.omegaOffsetAvg.y = m_gyro_biases.y();
            imuCalibration.omegaOffsetAvg.z = m_gyro_biases.z();
            imuHasValues                    = true;
        }

        if (m_imu_time_offset > 0) 
        {
            imuCalibration.timeOffsetMicros = m_imu_time_offset;
            imuHasValues                    = true;
        }

        if (imuCalibrationExists) 
        {
            m_calibration.updateImuCalibration(imuCalibration);
        }
        else if (imuHasValues) 
        {
            m_calibration.addImuCalibration(imuCalibration);
        }
    }

    void Capture::startCapture()
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning capture node...");
        auto times = m_reader.getTimeRange();

        const auto &live_capture = m_reader.getCameraCapturePtr();

        if (live_capture)
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
            RCLCPP_INFO(m_node->get_logger(), "Spinning camera info thread.");
            m_camera_info_thread = std::make_unique<std::thread>([this] 
            {
                rclcpp::Rate infoRate(25.0);
                while (m_spin_thread.load(std::memory_order_relaxed))
                {
                    const rclcpp::Time currentTime = dv_ros2_msgs::toRosTime(m_current_seek);
                    if (m_camera_info_publisher->get_subscription_count() > 0)
                    {
                        m_camera_info_msg.header.stamp = currentTime;
                        m_camera_info_publisher->publish(m_camera_info_msg);
                    }
                    if (m_imu_to_cam_transforms.has_value() && !m_imu_to_cam_transforms->transforms.empty())
                    {
                        m_imu_to_cam_transforms->transforms.back().header.stamp = currentTime;
                        m_transform_publisher->publish(*m_imu_to_cam_transforms);
                    }
                    infoRate.sleep();
                }
            });
        }
    }

    bool Capture::isRunning() const
    {
        return m_spin_thread.load(std::memory_order_relaxed);
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

    void Capture::runDiscovery(const std::string &syncServiceName)
    {
        const auto &liveCapture = m_reader.getCameraCapturePtr();

        if (liveCapture == nullptr)
        {
            return;
        }

        m_discovery_publisher = m_node->create_publisher<dv_ros2_msgs::msg::CameraDiscovery>("/dvs/discovery", 10);
        m_discovery_thread = std::make_unique<std::thread>([this, &liveCapture, &syncServiceName] 
        {
            dv_ros2_msgs::msg::CameraDiscovery message;
            message.is_master = liveCapture->isMasterCamera();
            message.name = liveCapture->getCameraName();
            message.startup_time = startup_time;
            message.publishing_events = m_params.events;
            message.publishing_frames = m_params.frames;
            message.publishing_imu = m_params.imu;
            message.publishing_triggers = m_params.triggers;
            message.sync_service_topic = syncServiceName;
            // 5 Hz is enough
            rclcpp::Rate rate(5.0);
            while (m_spin_thread)
            {
                if (m_discovery_publisher->get_subscription_count() > 0)
                {
                    // message.header.seq++; seq removed in ROS2
                    message.header.stamp = m_node->now();
                    m_discovery_publisher->publish(message);
                }
                rate.sleep();
            }
        });
    }

    std::map<std::string, std::string> Capture::discoverSyncDevices() const 
    {
        if (m_params.syncDeviceList.empty()) 
        {
            return {};
        }

        RCLCPP_INFO_STREAM(m_node->get_logger(), "Waiting for devices [" << fmt::format("{}", fmt::join(m_params.syncDeviceList, ", ")) << "] to be online...");

        // List info about each sync device
        struct DiscoveryContext 
        {
            std::map<std::string, std::string> serviceNames;
            std::atomic<bool> complete;
            std::vector<std::string> deviceList;

            void handleMessage(const dv_ros2_msgs::msg::CameraDiscovery::SharedPtr message) 
            {
                const std::string cameraName(message->name.c_str());
                if (serviceNames.contains(cameraName)) 
                {
                    return;
                }

                if (std::find(deviceList.begin(), deviceList.end(), cameraName) != deviceList.end()) 
                {
                    serviceNames.insert(std::make_pair(cameraName, message->sync_service_topic.c_str()));
                    if (serviceNames.size() == deviceList.size()) 
                    {
                        complete = true;
                    }
                }
            }
        };

        DiscoveryContext context;
        context.deviceList = m_params.syncDeviceList;
        context.complete   = false;

        auto subscriber = m_node->create_subscription<dv_ros2_msgs::msg::CameraDiscovery>("/dvs/discovery", 10, std::bind(&DiscoveryContext::handleMessage, &context, std::placeholders::_1));

        while (m_spin_thread.load(std::memory_order_relaxed) && !context.complete.load(std::memory_order_relaxed)) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        RCLCPP_INFO(m_node->get_logger(), "All sync devices are online.");

        return context.serviceNames;
    }

    void Capture::sendSyncCalls(const std::map<std::string, std::string> &serviceNames) const
    {
        if (serviceNames.empty()) 
        {
            return;
        }

        const auto &liveCapture = m_reader.getCameraCapturePtr();
        if (!liveCapture) 
        {
            return;
        }

        auto request = std::make_shared<dv_ros2_msgs::srv::SynchronizeCamera::Request>();
        request->timestamp_offset = liveCapture->getTimestampOffset();
        request->master_camera_name = liveCapture->getCameraName();

        for (const auto &[cameraName, serviceName] : serviceNames) 
        {
            if (serviceName.empty())
            {
                RCLCPP_ERROR_STREAM(m_node->get_logger(), "Camera [" << cameraName
                                            << "] can't be synchronized, synchronization service "
                                            "is unavailable, please check synchronization cable!");
                continue;
            }

            rclcpp::Client<dv_ros2_msgs::srv::SynchronizeCamera>::SharedPtr client = m_node->create_client<dv_ros2_msgs::srv::SynchronizeCamera>(serviceName);
            client->wait_for_service(std::chrono::seconds(1));
            auto result = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(m_node, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO_STREAM(m_node->get_logger(), "Camera [" << cameraName << "] is synchronized.");
            }
            else
            {
                RCLCPP_ERROR_STREAM(m_node->get_logger(), "Device [" << cameraName
                                            << "] failed to synchronize on service [" << serviceName << "]");
            }
        }   
    }

    void Capture::synchronizeCamera(const std::shared_ptr<rmw_request_id_t> request_header, 
                                    const std::shared_ptr<dv_ros2_msgs::srv::SynchronizeCamera::Request> req,
                                    std::shared_ptr<dv_ros2_msgs::srv::SynchronizeCamera::Response> rsp)
    {
        (void)request_header;
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Synchronization request received from [" << req->master_camera_name << "]");

        // assume failure case
        rsp->success = false;

        auto &liveCapture = m_reader.getCameraCapturePtr();
        if (!liveCapture) 
        {
            RCLCPP_WARN(m_node->get_logger(), "Received synchronization request on a non-live camera!");
            //return true;
        }
        if (liveCapture->isConnected() && liveCapture->isMasterCamera()) 
        {
            // Update the timestamp offset
            liveCapture->setTimestampOffset(req->timestamp_offset);
            RCLCPP_INFO_STREAM(m_node->get_logger(), "Camera [" << liveCapture->getCameraName() << "] synchronized: timestamp offset updated.");
            rsp->camera_name = liveCapture->getCameraName();
            rsp->success = true;
            m_synchronized = true;
        }
        else
        {
            RCLCPP_WARN(m_node->get_logger(), "Received synchronization request on a master camera, please check synchronization cable!");
        }
        //return true;
    }

    void Capture::synchronizationThread()
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning synchronization thread.");
        std::string serviceName;
        const auto &liveCapture = m_reader.getCameraCapturePtr();
        if (liveCapture->isMasterCamera()) 
        {
            RCLCPP_INFO_STREAM(m_node->get_logger(), "Camera [" << liveCapture->getCameraName() << "] is master camera.");
            // Wait for all cameras to show up
            const auto syncServiceList = discoverSyncDevices();
            runDiscovery(serviceName);
            sendSyncCalls(syncServiceList);
            m_synchronized = true;
        }
        else 
        {
            auto server_service = m_node->create_service<dv_ros2_msgs::srv::SynchronizeCamera>(fmt::format("{}/sync", liveCapture->getCameraName()), std::bind(&Capture::synchronizeCamera, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

            serviceName = server_service->get_service_name();
            runDiscovery(serviceName);

            // Wait for synchronization only if explicitly requested
            if (m_params.waitForSync)
            {
                m_synchronized = true;
            }

            size_t iterations = 0;
            while (m_spin_thread.load(std::memory_order_relaxed))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));

                // Do not print warnings if it's synchronized
                if (m_synchronized.load(std::memory_order_relaxed))
                {
                    continue;
                }

                if (iterations > 2000)
                {
                    RCLCPP_WARN(m_node->get_logger(), "[%s] waiting for synchronization service call...", liveCapture->getCameraName().c_str());
                    iterations = 0;
                }
                iterations++;
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
                        auto msg = dv_ros2_msgs::frameToRosImageMessage(*frame);
                        m_frame_publisher->publish(msg);
                    }

                    m_current_seek = frame->timestamp;

                    std::lock_guard<boost::recursive_mutex> lockGuard(m_reader_mutex);
                    frame = m_reader.getNextFrame();
                }
            });
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    void Capture::imuPublisher()
    {
        RCLCPP_INFO(m_node->get_logger(), "Spinning imu publisher.");

        std::optional<dv::cvector<dv::IMU>> imuData = std::nullopt;

        while(m_spin_thread)
        {
            m_imu_queue.consume_all([&](const int64_t timestamp)
            {
                if (!imuData.has_value())
                {
                    std::lock_guard<boost::recursive_mutex> lockGuard(m_reader_mutex);
                    imuData = m_reader.getNextImuBatch();
                }
                while (imuData.has_value() && !imuData->empty() && timestamp >= imuData->back().timestamp)
                {
                    if (m_imu_publisher->get_subscription_count() > 0)
                    {
                        for (auto &imu : *imuData)
                        {
                            imu.timestamp += m_imu_time_offset;
                            m_imu_publisher->publish(transformImuFrame(dv_ros2_msgs::toRosImuMessage(imu)));
                        }
                    }
                    m_current_seek = imuData->back().timestamp;

                    std::lock_guard<boost::recursive_mutex> lockGuard(m_reader_mutex);
                    imuData = m_reader.getNextImuBatch();
                }

                // If value present but empty, we don't want to keep it for later spins.
                if (imuData.has_value() && imuData->empty())
                {
                    imuData = std::nullopt;
                }
            });
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
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
                        auto msg = dv_ros2_msgs::toRosEventsMessage(store, resolution);
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

        std::optional<dv::cvector<dv::Trigger>> triggerData = std::nullopt;

        while (m_spin_thread)
        {
            m_trigger_queue.consume_all([&](const int64_t timestamp)
            {
                if (!triggerData.has_value())
                {
                    std::lock_guard<boost::recursive_mutex> lockGuard(m_reader_mutex);
                    triggerData = m_reader.getNextTriggerBatch();
                }
                while (triggerData.has_value() && !triggerData->empty() && timestamp >= triggerData->back().timestamp)
                {
                    if (m_trigger_publisher->get_subscription_count() > 0)
                    {
                        for (const auto &trigger : *triggerData)
                        {
                            m_trigger_publisher->publish(dv_ros2_msgs::toRosTriggerMessage(trigger));
                        }
                    }
                    m_current_seek = triggerData->back().timestamp;

                    std::lock_guard<boost::recursive_mutex> lockGuard(m_reader_mutex);
                    triggerData = m_reader.getNextTriggerBatch();
                }

                // If value present but empty, we don't want to keep it for later spins.
                if (triggerData.has_value() && triggerData->empty())
                {
                    triggerData = std::nullopt;
                }
            });
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }






}