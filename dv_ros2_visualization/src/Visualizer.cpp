#include "dv_ros2_visualization/Visualizer.hpp"

namespace dv_ros2_visualization
{
    Visualizer::Visualizer(const std::string &t_node_name) : Node(t_node_name), m_node{this}
    {
        RCLCPP_INFO(m_node->get_logger(), "Constructor is initialized.");
        parameterInitilization();

        if(!readParameters())
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameters.");
            rclcpp::shutdown();
            std::exit(EXIT_FAILURE);
        }

        parameterPrinter();

        m_slicer = std::make_unique<dv::EventStreamSlicer>();
        m_events_subscriber = m_node->create_subscription<dv_ros2_msgs::msg::EventArray>(
            "events", 10, std::bind(&Visualizer::eventCallback, this, std::placeholders::_1));
        m_frame_publisher = m_node->create_publisher<sensor_msgs::msg::Image>(m_params.image_topic, 10);

        RCLCPP_INFO(m_node->get_logger(), "Sucessfully launched.");
    }

    void Visualizer::start()
    {
        m_visualization_thread = std::thread(&Visualizer::visualize, this);
        RCLCPP_INFO(m_node->get_logger(), "Visualization thread is started.");
    }

    void Visualizer::stop()
    {
        RCLCPP_INFO(m_node->get_logger(), "Stopping the visualization node...");
        m_spin_thread = false;
        m_visualization_thread.join();
    }

    bool Visualizer::isRunning() const
    {
        return m_spin_thread.load(std::memory_order_relaxed);
    }

    void Visualizer::eventCallback(dv_ros2_msgs::msg::EventArray::SharedPtr events)
    {
        if (m_visualizer == nullptr)
        {
            m_visualizer = std::make_unique<dv::visualization::EventVisualizer>(cv::Size(events->width, events->height));
            updateConfiguration();
        }

        auto store = dv_ros2_msgs::toEventStore(*events);
        try
        {
            m_slicer->accept(store);
        }
        catch (std::out_of_range &e)
        {
            RCLCPP_WARN_STREAM(m_node->get_logger(), "Event out of range: " << e.what());
        }
    }

    void Visualizer::slicerCallback(const dv::EventStore &events)
    {
        m_event_queue.push(events);
    }

    void Visualizer::updateConfiguration()
    {
        if (m_job_id.has_value())
        {
            m_slicer->removeJob(m_job_id.value());
        }
        if (m_visualizer != nullptr)
        {
            m_visualizer->setBackgroundColor(cv::Scalar(m_params.background_color_b, m_params.background_color_g, m_params.background_color_r));
            m_visualizer->setPositiveColor(cv::Scalar(m_params.positive_event_color_b, m_params.positive_event_color_g, m_params.positive_event_color_r));
            m_visualizer->setNegativeColor(cv::Scalar(m_params.negative_event_color_b, m_params.negative_event_color_g, m_params.negative_event_color_r));
        }

        // convert frame_rate to ms (delta time)
        int32_t delta_time = static_cast<int>(1000 / m_params.frame_rate);
        m_job_id = m_slicer->doEveryTimeInterval(dv::Duration(delta_time * 1000LL), std::bind(&Visualizer::slicerCallback, this, std::placeholders::_1));        
    }

    void Visualizer::visualize()
    {
        RCLCPP_INFO(m_node->get_logger(), "Starting visualization.");
        while (m_spin_thread)
        {
            m_event_queue.consume_all([&](const dv::EventStore &events)
            {
                if (m_visualizer != nullptr)
                {
                    cv::Mat image = m_visualizer->generateImage(events);
                    sensor_msgs::msg::Image msg = dv_ros2_msgs::toRosImageMessage(image);
                    msg.header.stamp = dv_ros2_msgs::toRosTime(events.getLowestTime());
                    m_frame_publisher->publish(msg);
                }
            });
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    Visualizer::~Visualizer()
    {
        RCLCPP_INFO(m_node->get_logger(), "Destructor is activated.");
        stop();
        rclcpp::shutdown();
    }

    inline void Visualizer::parameterInitilization() const
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange int_range;
        rcl_interfaces::msg::FloatingPointRange float_range;

        m_node->declare_parameter("image_topic", m_params.image_topic);
        float_range.set__from_value(10.0).set__to_value(1000.0);
        descriptor.floating_point_range = {float_range};
        m_node->declare_parameter("frame_rate", m_params.frame_rate, descriptor);
        int_range.set__from_value(0).set__to_value(255).set__step(1);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("background_color_r", m_params.background_color_r, descriptor);
        m_node->declare_parameter("background_color_g", m_params.background_color_g, descriptor);
        m_node->declare_parameter("background_color_b", m_params.background_color_b, descriptor);
        m_node->declare_parameter("positive_event_color_r", m_params.positive_event_color_r, descriptor);
        m_node->declare_parameter("positive_event_color_g", m_params.positive_event_color_g, descriptor);
        m_node->declare_parameter("positive_event_color_b", m_params.positive_event_color_b, descriptor);
        m_node->declare_parameter("negative_event_color_r", m_params.negative_event_color_r, descriptor);
        m_node->declare_parameter("negative_event_color_g", m_params.negative_event_color_g, descriptor);
        m_node->declare_parameter("negative_event_color_b", m_params.negative_event_color_b, descriptor);
    }

    inline void Visualizer::parameterPrinter() const
    {
        RCLCPP_INFO(m_node->get_logger(), "-------- Parameters --------");
        RCLCPP_INFO(m_node->get_logger(), "image_topic: %s", m_params.image_topic.c_str());
        RCLCPP_INFO(m_node->get_logger(), "frame_rate: %f", m_params.frame_rate);
        RCLCPP_INFO(m_node->get_logger(), "background_color_r: %d", m_params.background_color_r);
        RCLCPP_INFO(m_node->get_logger(), "background_color_g: %d", m_params.background_color_g);
        RCLCPP_INFO(m_node->get_logger(), "background_color_b: %d", m_params.background_color_b);
        RCLCPP_INFO(m_node->get_logger(), "positive_event_color_r: %d", m_params.positive_event_color_r);
        RCLCPP_INFO(m_node->get_logger(), "positive_event_color_g: %d", m_params.positive_event_color_g);
        RCLCPP_INFO(m_node->get_logger(), "positive_event_color_b: %d", m_params.positive_event_color_b);
        RCLCPP_INFO(m_node->get_logger(), "negative_event_color_r: %d", m_params.negative_event_color_r);
        RCLCPP_INFO(m_node->get_logger(), "negative_event_color_g: %d", m_params.negative_event_color_g);
        RCLCPP_INFO(m_node->get_logger(), "negative_event_color_b: %d", m_params.negative_event_color_b);
    }

    inline bool Visualizer::readParameters()
    {
        if (!m_node->get_parameter("image_topic", m_params.image_topic))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter image_topic.");
            return false;
        }
        if (!m_node->get_parameter("frame_rate", m_params.frame_rate))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter frame_rate.");
            return false;
        }
        if (!m_node->get_parameter("background_color_r", m_params.background_color_r))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter background_color_r.");
            return false;
        }
        if (!m_node->get_parameter("background_color_g", m_params.background_color_g))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter background_color_g.");
            return false;
        }
        if (!m_node->get_parameter("background_color_b", m_params.background_color_b))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter background_color_b.");
            return false;
        }
        if (!m_node->get_parameter("positive_event_color_r", m_params.positive_event_color_r))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter positive_event_color_r.");
            return false;
        }
        if (!m_node->get_parameter("positive_event_color_g", m_params.positive_event_color_g))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter positive_event_color_g.");
            return false;
        }
        if (!m_node->get_parameter("positive_event_color_b", m_params.positive_event_color_b))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter positive_event_color_b.");
            return false;
        }
        if (!m_node->get_parameter("negative_event_color_r", m_params.negative_event_color_r))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter negative_event_color_r.");
            return false;
        }
        if (!m_node->get_parameter("negative_event_color_g", m_params.negative_event_color_g))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter negative_event_color_g.");
            return false;
        }
        if (!m_node->get_parameter("negative_event_color_b", m_params.negative_event_color_b))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read paramter negative_event_color_b.");
            return false;
        }
        return true;
    }

    rcl_interfaces::msg::SetParametersResult Visualizer::paramsCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        
        for (const auto &param : parameters)
        {
            if (param.get_name() == "image_topic")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
                {
                    m_params.image_topic = param.as_string();
                }
                else
                {
                    result.successful = false;
                    result.reason = "image_topic must be a string";
                }
            }
            else if (param.get_name() == "frame_rate")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    m_params.frame_rate = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "frame_rate must be a double";
                }
            }
            else if (param.get_name() == "background_color_r")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.background_color_r = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "background_color_r must be an integer";
                }
            }
            else if (param.get_name() == "background_color_g")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.background_color_g = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "background_color_g must be an integer";
                }
            }
            else if (param.get_name() == "background_color_b")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.background_color_b = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "background_color_b must be an integer";
                }
            }
            else if (param.get_name() == "positive_event_color_r")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.positive_event_color_r = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "positive_event_color_r must be an integer";
                }
            }
            else if (param.get_name() == "positive_event_color_g")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.positive_event_color_g = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "positive_event_color_g must be an integer";
                }
            }
            else if (param.get_name() == "positive_event_color_b")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.positive_event_color_b = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "positive_event_color_b must be an integer";
                }
            }
            else if (param.get_name() == "negative_event_color_r")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.negative_event_color_r = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "negative_event_color_r must be an integer";
                }
            }
            else if (param.get_name() == "negative_event_color_g")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.negative_event_color_g = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "negative_event_color_g must be an integer";
                }
            }
            else if (param.get_name() == "negative_event_color_b")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.negative_event_color_b = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "negative_event_color_b must be an integer";
                }
            }
            else
            {
                result.successful = false;
                result.reason = "unknown parameter";
            }
        }
        updateConfiguration();
        return result;
    }


} // namespace dv_ros2_visualization