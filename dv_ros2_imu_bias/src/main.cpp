#include "dv_ros2_messaging/messaging.hpp"
#include "dv_ros2_msgs/srv/set_imu_biases.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>


struct Params
{   
    /// @brief Variance threshold for bias estimation [m/s^2] [0, 2]
    double variance_threshold = 0.1;

    /// @brief Gravity range for bias estimation [m/s^2] [1, 2]
    double gravity_range = 1.0;

    /// @brief Duration of the collection phase [s] [1, 10]
    double collection_duration = 1.0;

    /// @brief Trigger for bias estimation
    bool estimate_biases = false;
};

Params params;
Eigen::Vector3f accBiases, gyroBiases;
rclcpp::Client<dv_ros2_msgs::srv::SetImuBiases>::SharedPtr imuBiasClient;
std::shared_ptr<rclcpp::Node> imuBiasNode;
std::vector<std::vector<float>> accValues(3), gyroValues(3);
int64_t startTimestamp = -1;
bool startCollecting = false;


rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "variance_threshold")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                params.variance_threshold = param.as_double();
            }
            else
            {
                result.successful = false;
                result.reason = "variance_threshold must be a double";
            }
        }
        else if (param.get_name() == "gravity_range")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                params.gravity_range = param.as_double();
            }
            else
            {
                result.successful = false;
                result.reason = "gravity_range must be a double";
            }
        }
        else if (param.get_name() == "collection_duration")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                params.collection_duration = param.as_double();
            }
            else
            {
                result.successful = false;
                result.reason = "collection_duration must be a double";
            }
        }
        else if (param.get_name() == "estimate_biases")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                params.estimate_biases = param.as_bool();
                startCollecting = !startCollecting;
            }
            else
            {
                result.successful = false;
                result.reason = "estimate_biases must be a bool";
            }
        }
        else
        {
            result.successful = false;
            result.reason = "unknown parameter";
        }
    }
    return result;
}

void storeImuBiases()
{
    RCLCPP_INFO(imuBiasNode->get_logger(), "Attempting to store biases...");
    auto request = std::make_shared<dv_ros2_msgs::srv::SetImuBiases::Request>();
    request->acc_biases.x = accBiases.x();
    request->acc_biases.y = accBiases.y();
    request->acc_biases.z = accBiases.z();

    request->gyro_biases.x = gyroBiases.x();
    request->gyro_biases.y = gyroBiases.y();
    request->gyro_biases.z = gyroBiases.z();

    auto result = imuBiasClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(imuBiasNode, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(imuBiasNode->get_logger(), "Biases stored, shutting down node");
        rclcpp::shutdown();
    }
    else
    {
        RCLCPP_ERROR(imuBiasNode->get_logger(), "Setting IMU biases failed, please investigate any issues in the log and retry.");
    }
}

void estimateBias()
{
    const Eigen::Vector3f earthG(0.f, 9.81007f, 0.f);

    // Validate standard deviation to make sure the data device was stable during
	// collection
    std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>> values;

    // Mapping the data
	for (const auto &acc : accValues) {
		Eigen::VectorXf data = Eigen::VectorXf::Map(acc.data(), static_cast<Eigen::Index>(acc.size()));
		values.push_back(data);
	}
	for (const auto &gyro : gyroValues) {
		Eigen::VectorXf data = Eigen::VectorXf::Map(gyro.data(), static_cast<Eigen::Index>(gyro.size()));
		values.push_back(data);
	}

	std::vector<float> biases;
	// Sanity checking and mean estimation
	for (auto &sequence : values) {
		const float mean = sequence.mean();
		const float variance
			= std::sqrt((sequence.array() - mean).square().sum() / static_cast<float>(sequence.size() - 1));

		if (variance > params.variance_threshold) {
			throw dv::exceptions::RuntimeError("Some motion detected in IMU data, please keep the device steady while collecting and repeat.");
		}

		biases.push_back(mean);
	}

    if (biases[1] < -(earthG[1] + params.gravity_range) || biases[1] > -(earthG[1] - params.gravity_range)) {
		throw dv::exceptions::RuntimeError(
			"Gravity vector is not aligned, please make sure to place the camera horizontally on a level surface.");
	}

    accBiases  = Eigen::Vector3f(biases[0], biases[1], biases[2]) + earthG;
	gyroBiases = Eigen::Vector3f(biases[3], biases[4], biases[5]);

    //Result printing
    RCLCPP_INFO(imuBiasNode->get_logger(), "Bias estimation was successful!");
    RCLCPP_INFO_STREAM(imuBiasNode->get_logger(), "Accelerometer biases [x, y, z] in m/s^2:" << "[" << accBiases.x() << ", " << accBiases.y() << ", " << accBiases.z() << "]");
    RCLCPP_INFO_STREAM(imuBiasNode->get_logger(), "Gyroscope biases [x, y, z] in rad/s:" << "[" << gyroBiases.x() << ", " << gyroBiases.y() << ", " << gyroBiases.z() << "]");
    RCLCPP_INFO_STREAM(imuBiasNode->get_logger(), "Earth gravity vector: [" << earthG.x() << ", " << earthG.y() << ", " << earthG.z() << "]");
}

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!startCollecting)
    {
        return;
    }
    if (startTimestamp < 0)
    {
        startTimestamp = dv_ros2_msgs::toDvTime(msg->header.stamp);
    }
    accValues[0].push_back(static_cast<float>(msg->linear_acceleration.x));
    accValues[1].push_back(static_cast<float>(msg->linear_acceleration.y));
    accValues[2].push_back(static_cast<float>(msg->linear_acceleration.z));

    gyroValues[0].push_back(static_cast<float>(msg->angular_velocity.x));
    gyroValues[1].push_back(static_cast<float>(msg->angular_velocity.y));
    gyroValues[2].push_back(static_cast<float>(msg->angular_velocity.z));

    if (dv_ros2_msgs::toDvTime(msg->header.stamp) - startTimestamp > static_cast<int64_t>(params.collection_duration * 1e+6))
    {
        startCollecting = false;
        const size_t sampleSize = accValues[0].size();
        RCLCPP_INFO_STREAM(imuBiasNode->get_logger(), "Collected " << sampleSize << " samples over " << static_cast<double>(dv_ros2_msgs::toDvTime(msg->header.stamp) - startTimestamp) * 1e-6 << " seconds");

        estimateBias();
        storeImuBiases();
    }

}

void parameterInitilization()
{
    imuBiasNode->declare_parameter("variance_threshold", params.variance_threshold);
    imuBiasNode->declare_parameter("gravity_range", params.gravity_range);
    imuBiasNode->declare_parameter("collection_duration", params.collection_duration);
    imuBiasNode->declare_parameter("estimate_biases", params.estimate_biases);
}

void parameterPrinter()
{
    RCLCPP_INFO(imuBiasNode->get_logger(), "---- Parameters ----");
    RCLCPP_INFO(imuBiasNode->get_logger(), "variance_threshold: %f", params.variance_threshold);
    RCLCPP_INFO(imuBiasNode->get_logger(), "gravity_range: %f", params.gravity_range);
    RCLCPP_INFO(imuBiasNode->get_logger(), "collection_duration: %f", params.collection_duration);
    RCLCPP_INFO(imuBiasNode->get_logger(), "estimate_biases: %s", params.estimate_biases ? "true" : "false");
}

bool readParameters()
{
    if (!imuBiasNode->get_parameter("variance_threshold", params.variance_threshold))
    {
        RCLCPP_ERROR(imuBiasNode->get_logger(), "Failed to get parameter variance_threshold");
        return false;
    }
    if (!imuBiasNode->get_parameter("gravity_range", params.gravity_range))
    {
        RCLCPP_ERROR(imuBiasNode->get_logger(), "Failed to get parameter gravity_range");
        return false;
    }
    if (!imuBiasNode->get_parameter("collection_duration", params.collection_duration))
    {
        RCLCPP_ERROR(imuBiasNode->get_logger(), "Failed to get parameter collection_duration");
        return false;
    }
    if (!imuBiasNode->get_parameter("estimate_biases", params.estimate_biases))
    {
        RCLCPP_ERROR(imuBiasNode->get_logger(), "Failed to get parameter estimate_biases");
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string t_node_name("dv_ros2_imu_bias");

    imuBiasNode = std::make_shared<rclcpp::Node>(t_node_name);

    RCLCPP_INFO(imuBiasNode->get_logger(), "Keep the camera steady on a level surface and enable estimate_biases in the rqt_reconfigure UI...");
    
    parameterInitilization();

    if (!readParameters())
    {
        RCLCPP_ERROR(imuBiasNode->get_logger(), "Failed to read parameters");
        rclcpp::shutdown();
        std::exit(EXIT_FAILURE);
    }

    parameterPrinter();

    imuBiasClient = imuBiasNode->create_client<dv_ros2_msgs::srv::SetImuBiases>("set_imu_biases");
    imuBiasClient->wait_for_service(std::chrono::seconds(1));

    auto imuSubscriber = imuBiasNode->create_subscription<sensor_msgs::msg::Imu>("imu", 10, imuCallback);


    auto cb_handle = imuBiasNode->add_on_set_parameters_callback(paramsCallback);
    

    rclcpp::spin(imuBiasNode);
    rclcpp::shutdown();
    return 0;

}

