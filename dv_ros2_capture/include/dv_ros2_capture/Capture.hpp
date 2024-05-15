#ifndef _CAPTURE_HPP_
#define _CAPTURE_HPP_

// C++ System Headers
#include <iostream>
#include <vector>
#include <map>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <boost/thread/thread.hpp>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"


namespace dv_ros2_capture
{
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

    private:
        /// @brief Parameter initialization
        inline void parameterInitilization() const;

        /// @brief Parameter reader
        inline void rclcppParameterReader();

        /// @brief Print parameters
        inline void parameterPrinter() const;

        /// @brief Reads the std library variables and ROS2 parameters
        /// @return true if all parameters are read successfully
        inline bool readParameters();

        /// @brief Continous callback function for publishing
        inline void continousCallback();

        /// @brief eclcpp node variable
        rclcpp::Node::SharedPtr m_node;

        /// @brief Timer for continous callback
        rclcpp::TimerBase::SharedPtr m_timer;

        /// @brief Rate for publishing
        double m_rate{};

        /// @brief Rate for waiting in continous callback
        const double m_rate_waiting{};

        /// @brief Parameter for rate for continous callback in Hz
        rclcpp::Parameter m_parameter_rate;

    };
} // namespace dv_ros2_capture

#endif // _CAPTURE_HPP_
