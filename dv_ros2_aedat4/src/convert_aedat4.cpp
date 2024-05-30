#include <dv-processing/io/mono_camera_recording.hpp>

#include <dv_ros2_messaging/messaging.hpp>

#include <CLI/CLI.hpp>

#include <rosbag2_cpp/writer.hpp>

#include <filesystem>

void progressBar(const float progress)
{
   	int barWidth = 70;

	std::cout << "[";
	int pos = static_cast<int>(static_cast<float>(barWidth) * progress);
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos)
			std::cout << "=";
		else if (i == pos)
			std::cout << ">";
		else
			std::cout << " ";
	}
	std::cout << "] " << int(progress * 100.0) << " %\r";
	std::cout.flush();
}

int main(int argc, char **argv) {
	namespace fs = std::filesystem;

	CLI::App app{"Aedat4 to rosbag converter. Supports single camera recordings with events, images, imu, and trigger "
				 "data streams."};

	std::string input;
	std::string output;
	std::string topicNamespace = "/recording";
	bool overwrite             = false;
	bool verbose               = false;
	app.add_option("-i,--input", input, "Input aedat4 file")->required()->check(CLI::ExistingFile);
	app.add_option("-o,--output", output,
		"Output rosbag file path, if not provided, will create file with same path as input, but with .bag extension");
	app.add_flag("-n,--namespace", topicNamespace, "Topic namespace");
	app.add_flag("-f,--force-overwrite", overwrite);
	app.add_flag("-v,--verbose", verbose);

	CLI11_PARSE(app, argc, argv);

	dv::io::MonoCameraRecording reader(input);

	if (output.empty()) {
		output = fs::path(input).replace_extension(".bag").string();
	}

	if (fs::exists(output) && !overwrite) {
		std::cout << "Output file already exists. Do you want to overwrite? [y/n]" << std::endl;
		char in;
		std::cin >> in;
		if (in != 'y') {
			std::cout << "Exiting due to user input" << std::endl;
			return EXIT_SUCCESS;
		}
	}

	auto bag = std::make_unique<rosbag2_cpp::Writer>();
    bag->open(output);

	dv::io::DataReadHandler handler;
	if (reader.isFrameStreamAvailable()) {
	}

	handler.mFrameHandler = [&bag, &topicNamespace](const dv::Frame &frame) {
		auto msg = dv_ros2_msgs::frameToRosImageMessage(frame);
		bag->write(msg, topicNamespace + "/image", msg.header.stamp);
	};

	if (reader.isEventStreamAvailable()) {
		cv::Size resolution = reader.getEventResolution().value();

		handler.mEventHandler = [&bag, &topicNamespace, &resolution](const dv::EventStore &events) {
			if (events.isEmpty()) {
				return;
			}

			auto msg = dv_ros2_msgs::toRosEventsMessage(events, resolution);
			bag->write(msg, topicNamespace + "/events", dv_ros2_msgs::toRosTime(events.getLowestTime()));
		};
	}

	handler.mImuHandler = [&bag, &topicNamespace](const dv::cvector<dv::IMU> &imuBatch) {
		for (const auto &imu : imuBatch) {
			auto msg = dv_ros2_msgs::toRosImuMessage(imu);
			bag->write(msg, topicNamespace + "/imu", msg.header.stamp);
		}
	};

	handler.mTriggersHandler = [&bag, &topicNamespace](const dv::cvector<dv::Trigger> &triggerBatch) {
		for (const auto &trigger : triggerBatch) {
			auto msg = dv_ros2_msgs::toRosTriggerMessage(trigger);
			bag->write(msg, topicNamespace + "/trigger", msg.timestamp);
		}
	};

	const auto startEnd = reader.getTimeRange();
	const auto duration = static_cast<float>(startEnd.second - startEnd.first);
	while (reader.handleNext(handler)) {
		const float progress = static_cast<float>((handler.seek - startEnd.first)) / duration;
		if (verbose) {
			progressBar(progress);
		}
	}

	bag->close();

	return EXIT_SUCCESS;
}

