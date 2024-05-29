#include "dv_ros2_runtime_modules/Ros2EventBridge.hpp"

void Ros2EventBridge::initInputs(dv::InputDefinitionList &in)
{
    in.addEventInput("events");
    in.addTriggerInput("sync", true);
}

const char *Ros2EventBridge::initDescription()
{
    return "Publishes event stream into ROS2";
}

bool Ros2EventBridge::loadCalibrationFile(const std::string &filename)
{
    // loads single camera calibration file
    if (filename.empty())
    {
        log.error << "No camera calibration file specified." << std::endl;
        return (false);
    }

    cv::FileStorage fs(filename, cv::FileStorage::READ);

	if (!fs.isOpened()) 
    {
		log.error << "Impossible to load the camera calibration file: " << filename << std::endl;
		return (false);
	}
	auto typeNode = fs["type"];

	if (!cvExists(typeNode) || !typeNode.isString()) 
    {
		log.info << "Old-style camera calibration file found." << std::endl;

		if (!cvExists(fs["camera_matrix"]) || !cvExists(fs["distortion_coefficients"])) 
        {
			log.error.format("Calibration data not present in file: %s", filename);
			return (false);
		}

		fs["camera_matrix"] >> camera_mat;
		fs["distortion_coefficients"] >> distortion_coeffs;

		log.info.format("Loaded camera matrix and distortion coefficients from file: %s", filename);
	}
	else 
    {
		log.info << "New-style camera calibration file found." << std::endl;

		auto cameraNode = fs[camera_id];

		if (!cvExists(cameraNode) || !cameraNode.isMap()) 
        {
			log.error.format("Calibration data for camera %s not present in file: %s", camera_id, filename);
			return (false);
		}

		if (!cvExists(cameraNode["camera_matrix"]) || !cvExists(cameraNode["distortion_coefficients"])) 
        {
			log.error.format("Calibration data for camera %s not present in file: %s", camera_id, filename);
			return (false);
		}

		cameraNode["camera_matrix"] >> camera_mat;
		cameraNode["distortion_coefficients"] >> distortion_coeffs;

		log.info.format(
			"Loaded camera matrix and distortion coefficients for camera %s from file: %s", camera_id, filename);
	}

	// Only fisheye is different, let's keep this by default for now
	camera_info.distortion_model = "plumb_bob";

	// Copy camera_mat by value
	for (int i = 0; i < camera_mat.rows; i++) {
		for (int j = 0; j < camera_mat.cols; j++) {
			camera_info.k.at(i * camera_mat.rows + j)       = camera_mat.at<double>(i, j);
			camera_info.p.at(i * (camera_mat.rows + 1) + j) = camera_mat.at<double>(i, j);
		}
	}

	camera_info.p.at(11) = 1.0;

	for (int i = 0; i < distortion_coeffs.rows; i++) {
		for (int j = 0; j < distortion_coeffs.cols; j++) {
			camera_info.d.push_back(distortion_coeffs.at<double>(i, j));
		}
	}

	// Identity matrix
	camera_info.r.at(0) = 1.0;
	camera_info.r.at(4) = 1.0;
	camera_info.r.at(8) = 1.0;

	return true;
}

void Ros2EventBridge::publishEventsMsg(const dv_ros2_msgs::msg::EventArray &msg)
{
    event_pub->publish(msg);
    if (camera_info_pub)
    {
        camera_info.header.stamp = msg.header.stamp;
        camera_info_pub->publish(camera_info);
    }
    rclcpp::spin_some(node);
}

void Ros2EventBridge::run()
{
    auto events = inputs.getEventInput("events");

    if (!event_pub)
    {
        // Read all configuration, initialize the ROS2 connection.
        if (!rclcpp::ok())
        {
            char **argv = {nullptr};
            int argc = 0;
            rclcpp::init(argc, argv);
        }

        node = std::make_shared<rclcpp::Node>("DVS_Event_Publisher"); 

        std::string topicNS = config.getString("topicNamespace");

        event_pub = node->create_publisher<dv_ros2_msgs::msg::EventArray>(topicNS + "/events", 10);

        // Load calibration from file.
        setCameraID(events.getOriginDescription());
        if (loadCalibrationFile(config.getString("calibrationFile")))
        {
            camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(topicNS + "/camera_info", 1);
            camera_info.width = static_cast<unsigned int>(events.sizeX());
            camera_info.height = static_cast<unsigned int>(events.sizeY());
        }
    }

    auto sync = inputs.getTriggerInput("sync");
    if (sync.isConnected())
    {
        if (auto e = events.events())
        {
            events_buffer.push_back(std::make_shared<DvEvents>(e));
            width = static_cast<uint16_t>(events.sizeX());
            height = static_cast<uint16_t>(events.sizeY());
        }
        if (sync.data() && !events_buffer.empty())
        {
            int64_t timestamp = sync.data().front().timestamp;
            auto closestEvents = findClosest(timestamp);
            dv::EventStore store(**closestEvents);

            auto eventsArray = dv_ros2_msgs::toRosEventsMessage(store, cv::Size(width, height));
            publishEventsMsg(eventsArray);
        }
    }
    else
    {
        dv::EventStore store(events.events().getBasePointer());
        auto eventsArray = dv_ros2_msgs::toRosEventsMessage(store, cv::Size(width, height));

        publishEventsMsg(eventsArray);
    }
}

void Ros2EventBridge::configUpdate()
{
    uint32_t bufferSize = static_cast<uint32_t>(config.getInt("bufferSize"));
    events_buffer = boost::circular_buffer<DvEventsPtr>(bufferSize);
    ModuleBase::configUpdate();
}

void Ros2EventBridge::initConfigOptions(dv::RuntimeConfig &config) {
	config.add("topicNamespace", dv::ConfigOption::stringOption("ROS Topic name", "/dvxplorer"));
	config.add("calibrationFile",
		dv::ConfigOption::fileOpenOption(
			"The name of the file from which to load the calibration settings for undistortion.", "xml"));
	config.add("bufferSize", dv::ConfigOption::intOption("Size of the circular buffer", 10, 1, 1000));

	config.setPriorityOptions({"calibrationFile", "topicNamespace"});
}

bool Ros2EventBridge::cvExists(const cv::FileNode &fn) {
    return fn.type() != cv::FileNode::NONE;
}

void Ros2EventBridge::setCameraID(const std::string &originDescription)
{
    // Camera origin descriptions are fine, never start with a digit or have spaces
	// or other special characters in them that fail with OpenCV's FileStorage.
	// So we just clean/escape the string for possible other sources.
	auto str = std::regex_replace(originDescription, filenameCleanupRegex, "_");

	// FileStorage node names can't start with - or 0-9.
	// Prefix with an underscore in that case.
	if ((str[0] == '-') || (std::isdigit(str[0]) != 0)) {
		str = "_" + str;
	}

	camera_id = str;
}

Ros2EventBridge::EventsBuffer::const_iterator Ros2EventBridge::findClosest(int64_t timestamp) const
{
    int64_t minDistance = INT64_MAX;
    auto iter = events_buffer.begin();
    auto minIter = events_buffer.end();
    while (iter != events_buffer.end())
    {
        int64_t timedistance = std::abs((*iter)->front().timestamp() - timestamp);
        if (timedistance < minDistance)
        {
            minIter = iter;
            minDistance = timedistance;
        }
        iter++;
    }
    return minIter;
}

registerModuleClass(Ros2EventBridge)