#include "dv_ros2_accumulation/Accumulator.h"

namespace dv_ros2_accumulation
{
    Accumulator::Accumulator(const std::string &t_node_name)
    : Node(t_node_name), m_node{this},
        m_publisher_topic_image{}, m_rate{20.0}, m_rate_waiting{1.0},
        m_parameter_publisher_topic_image{}
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

        m_publishers_image[m_publisher_topic_image] = m_node->create_publisher<sensor_msgs::msg::Image>(
            m_publisher_topic_image, 10);

        m_timer = m_node->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / m_rate)),
                                            std::bind(&Accumulator::continousCallback, this));

        RCLCPP_INFO(m_node->get_logger(), "Successfully launched.");
    }

   
    Accumulator::~Accumulator()
    {
        RCLCPP_INFO(m_node->get_logger(), "Destructor is activated. ");

        rclcpp::shutdown();
    }

    inline void Accumulator::parameterInitilization() const
    {
       m_node->declare_parameter("publisher_topic_image", "image");
       m_node->declare_parameter("rate", 20.0);
    }

    inline void Accumulator::rclcppParameterReader()
    {
        m_parameter_publisher_topic_image = m_node->get_parameter("publisher_topic_image");
        m_parameter_rate = m_node->get_parameter("rate");
    }

    inline void Accumulator::parameterPrinter() const
    {
        RCLCPP_INFO(m_node->get_logger(), "publisher_topic_image: %s", m_parameter_publisher_topic_image.as_string().c_str());
        RCLCPP_INFO(m_node->get_logger(), "rate: %f", m_parameter_rate.as_double());
    }

    inline bool Accumulator::readParameters()
    {
        if (!m_node->get_parameter("publisher_topic_image", m_publisher_topic_image))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter publisher_topic_image");
            return false;
        }
        if (!m_node->get_parameter("rate", m_rate))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter rate");
            return false;
        }
       
        return true;
    }

    inline void Accumulator::continousCallback()
    {
        RCLCPP_INFO(m_node->get_logger(), "continousCallback is activated. ");
        return;
    }
}  // namespace dv_ros2_accumulation