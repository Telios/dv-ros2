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
#include <boost/thread/thread.hpp>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dv_ros2_accumulation
{
    class Accumulator : public rclcpp::Node
    {
        using rclcpp::Node::Node;
    public:
        /// @brief Default constructor
        /// @param t_node_name name of the node
        Accumulator(const std::string &t_node_name);

        /// @brief Shallow copy constructor
        /// @param source to copy from
        Accumulator(const Accumulator &source);

        /// @brief Destructor
        ~Accumulator();

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
        
        /// @brief Publishers for Image messages
        std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> m_publishers_image;

        /// @brief Timer for continous callback
        rclcpp::TimerBase::SharedPtr m_timer;

        /// @brief Publisher topic for Image messages
        std::string m_publisher_topic_image;

        /// @brief Rate for publishing
        double m_rate{};

        /// @brief Rate for waiting in continous callback
        const double m_rate_waiting{};

        /// @brief Parameter for publisher topic for Image messages
        rclcpp::Parameter m_parameter_publisher_topic_image;

        /// @brief Parameter for rate for continous callback in Hz
        rclcpp::Parameter m_parameter_rate;

    };
}  // namespace dv_ros2_accumulation