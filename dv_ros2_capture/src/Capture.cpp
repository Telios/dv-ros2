#include "dv_ros2_capture/Capture.hpp"

namespace dv_ros2_capture
{
    Capture::Capture(const std::string &t_node_name) 
    : Node(t_node_name), m_node{this},
        m_rate{20.0}, m_rate_waiting{1.0}, m_parameter_rate{}
    {
        RCLCPP_INFO(m_node->get_logger(), "Constructor is initialized");
        parameterInitilization();
        rclcppParameterReader();

        if (!readParameters())
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameters");
            rclcpp::shutdown();
            std::exit(EXIT_FAILURE);
        }

        parameterPrinter();
        m_timer = m_node->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / m_rate)),
                                            std::bind(&Capture::continousCallback, this));

        RCLCPP_INFO(m_node->get_logger(), "Successfully launched.");
    }

    Capture::~Capture()
    {
        RCLCPP_INFO(m_node->get_logger(), "Destructor is initialized");
        rclcpp::shutdown();
    }

    inline void Capture::parameterInitilization() const
    {
        m_node->declare_parameter("rate", 20.0);
    }

    inline void Capture::rclcppParameterReader()
    {
        m_parameter_rate = m_node->get_parameter("rate");
    }

    inline void Capture::parameterPrinter() const
    {
        RCLCPP_INFO(m_node->get_logger(), "Rate: %f", m_rate);
    }

    inline bool Capture::readParameters()
    {
        if (!m_node->get_parameter("rate", m_rate))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter rate");
            return false;
        }
        return true;
    }

    inline void Capture::continousCallback()
    {
        RCLCPP_INFO(m_node->get_logger(), "Continous callback is called");
    }


}