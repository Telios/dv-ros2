#include "dv_ros2_accumulation/Accumulator.h"

namespace dv_ros2_accumulation
{
    Accumulator::Accumulator(const std::string &t_node_name)
    : Node(t_node_name), m_node{this}
    {
        RCLCPP_INFO(m_node->get_logger(), "Constructor is initialized");
        parameterInitilization();

        if (!readParameters())
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameters");
            rclcpp::shutdown();
            std::exit(EXIT_FAILURE);
        }

        parameterPrinter();

        m_events_subscriber = m_node->create_subscription<dv_ros2_msgs::msg::EventArray>("events", 10, std::bind(&Accumulator::eventCallback, this, std::placeholders::_1));
        m_frame_publisher = m_node->create_publisher<sensor_msgs::msg::Image>("image", 10);
        m_slicer = std::make_unique<dv::EventStreamSlicer>();

        RCLCPP_INFO(m_node->get_logger(), "Successfully launched.");
    }

    void Accumulator::start()
    {
        m_accumulation_thread = std::thread(&Accumulator::accumulate, this);
        RCLCPP_INFO(m_node->get_logger(), "Accumulation started");
    }

    void Accumulator::stop()
    {
        RCLCPP_INFO(m_node->get_logger(), "Stopping the accumulation node...");
        m_spin_thread = false;
        m_accumulation_thread.join();
    }

    bool Accumulator::isRunning() const
    {
        return m_spin_thread.load(std::memory_order_relaxed);
    }

    void Accumulator::eventCallback(dv_ros2_msgs::msg::EventArray::SharedPtr events)
    {
            if (m_accumulator == nullptr)
            {
                m_accumulator = std::make_unique<dv::Accumulator>(cv::Size(events->width, events->height));
                updateConfiguration();
            }
        
            if (m_accumulator_edge == nullptr)
            {
                m_accumulator_edge = std::make_unique<dv::EdgeMapAccumulator>(cv::Size(events->width, events->height));
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

    void Accumulator::slicerCallback(const dv::EventStore &events)
    {
        m_event_queue.push(events);
    }

    void Accumulator::updateConfiguration()
    {
        if (m_job_id.has_value())
        {
            m_slicer->removeJob(m_job_id.value());
        }

        if (m_params.accumulation_mode == "FRAME" && m_accumulator != nullptr)
        {
            m_accumulator->setEventContribution(m_params.event_contribution);
            m_accumulator->setDecayParam(m_params.decay_param);
            m_accumulator->setMinPotential(m_params.min_potential);
            m_accumulator->setMaxPotential(m_params.max_potential);
            m_accumulator->setNeutralPotential(m_params.neutral_potential);
            m_accumulator->setIgnorePolarity(m_params.rectify_polarity);
            m_accumulator->setSynchronousDecay(m_params.synchronous_decay);
            m_accumulator->setDecayFunction(static_cast<dv::Accumulator::Decay>(m_params.decay_function));
        }
        else if (m_params.accumulation_mode == "EDGE" && m_accumulator_edge != nullptr)
        {
            if (m_params.enable_decay)
            {
                m_accumulator_edge->setDecay(static_cast<float>(m_params.decay_edge));
            }
            else
            {
                m_accumulator_edge->setDecay(-1.f);
            }
            m_accumulator_edge->setIgnorePolarity(m_params.rectify_polarity);
            m_accumulator_edge->setEventContribution(static_cast<float>(m_params.event_contribution));
            m_accumulator_edge->setNeutralPotential(static_cast<float>(m_params.neutral_potential));
        }
        if (m_params.accumulation_mode != "FRAME" && m_params.accumulation_mode != "EDGE")
        {
            throw dv::exceptions::InvalidArgument<std::string>("Unknown accumulation mode", m_params.accumulation_mode);
        }

        switch (m_params.slice_method)
        {
            case static_cast<int>(SliceMethod::TIME):
            {
                m_job_id = m_slicer->doEveryTimeInterval(dv::Duration(m_params.accumulation_time * 1000LL), std::bind(&Accumulator::slicerCallback, this, std::placeholders::_1));
                break;
            }
            case static_cast<int>(SliceMethod::NUMBER):
            {
                m_job_id = m_slicer->doEveryNumberOfElements(m_params.accumulation_number, std::bind(&Accumulator::slicerCallback, this, std::placeholders::_1));
                break;
            }
            default:
            {
                throw dv::exceptions::InvalidArgument<int>("Unknown slicing method id", m_params.slice_method);
            }
        }
    }

    void Accumulator::accumulate()
    {
        RCLCPP_INFO(m_node->get_logger(), "Starting accumulation.");
        
        while (m_spin_thread)
        {
            if (m_params.accumulation_mode == "FRAME" ? m_accumulator != nullptr : m_accumulator_edge != nullptr)
            {
                m_event_queue.consume_all([&](const dv::EventStore &events)
                {
                    m_params.accumulation_mode == "FRAME" ? m_accumulator->accumulate(events) : m_accumulator_edge->accumulate(events);
                    dv::Frame frame = m_params.accumulation_mode == "FRAME" ? m_accumulator->generateFrame() : m_accumulator_edge->generateFrame();
                    sensor_msgs::msg::Image msg = dv_ros2_msgs::toRosImageMessage(frame.image);
                    msg.header.stamp = dv_ros2_msgs::toRosTime(frame.timestamp);
                    m_frame_publisher->publish(msg);
                });
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }



   
    Accumulator::~Accumulator()
    {
        RCLCPP_INFO(m_node->get_logger(), "Destructor is activated. ");
        stop();
        rclcpp::shutdown();
    }

    inline void Accumulator::parameterInitilization() const
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange int_range;
        rcl_interfaces::msg::FloatingPointRange float_range;
        int_range.set__from_value(1).set__to_value(1000).set__step(1);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("accumulation_time", m_params.accumulation_time, descriptor);
        int_range.set__from_value(1000).set__to_value(10000000).set__step(1);
        descriptor.integer_range = {int_range};
        m_node->declare_parameter("accumulation_number", m_params.accumulation_number, descriptor);
        m_node->declare_parameter("synchronous_decay", m_params.synchronous_decay);
        float_range.set__from_value(0.0).set__to_value(1.0);
        descriptor.floating_point_range = {float_range};
        m_node->declare_parameter("min_potential", m_params.min_potential, descriptor);
        m_node->declare_parameter("max_potential", m_params.max_potential, descriptor);
        m_node->declare_parameter("neutral_potential", m_params.neutral_potential, descriptor);
        m_node->declare_parameter("event_contribution", m_params.event_contribution, descriptor);
        m_node->declare_parameter("rectify_polarity", m_params.rectify_polarity);
        float_range.set__from_value(0.0).set__to_value(1e+10);
        descriptor.floating_point_range = {float_range};
        m_node->declare_parameter("decay_param", m_params.decay_param, descriptor);
        m_node->declare_parameter("slice_method", m_params.slice_method);
        m_node->declare_parameter("decay_function", m_params.decay_function);
        m_node->declare_parameter("accumulation_mode", m_params.accumulation_mode);
        m_node->declare_parameter("enable_decay", m_params.enable_decay);
        float_range.set__from_value(0.0).set__to_value(1.0);
        descriptor.floating_point_range = {float_range};
        m_node->declare_parameter("decay_edge", m_params.decay_edge, descriptor);
    }

    inline void Accumulator::parameterPrinter() const
    {
        RCLCPP_INFO(m_node->get_logger(), "-------- Parameters --------");
        RCLCPP_INFO(m_node->get_logger(), "accumulation_time: %d", m_params.accumulation_time);
        RCLCPP_INFO(m_node->get_logger(), "accumulation_number: %d", m_params.accumulation_number);
        RCLCPP_INFO(m_node->get_logger(), "synchronous_decay: %s", m_params.synchronous_decay ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "min_potential: %f", m_params.min_potential);
        RCLCPP_INFO(m_node->get_logger(), "max_potential: %f", m_params.max_potential);
        RCLCPP_INFO(m_node->get_logger(), "neutral_potential: %f", m_params.neutral_potential);
        RCLCPP_INFO(m_node->get_logger(), "event_contribution: %f", m_params.event_contribution);
        RCLCPP_INFO(m_node->get_logger(), "rectify_polarity: %s", m_params.rectify_polarity ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "decay_param: %f", m_params.decay_param);
        RCLCPP_INFO(m_node->get_logger(), "slice_method: %d", m_params.slice_method);
        RCLCPP_INFO(m_node->get_logger(), "decay_function: %d", m_params.decay_function);
        RCLCPP_INFO(m_node->get_logger(), "accumulation_mode: %s", m_params.accumulation_mode.c_str());
        RCLCPP_INFO(m_node->get_logger(), "enable_decay: %s", m_params.enable_decay ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "decay_edge: %s", m_params.decay_edge ? "true" : "false");
        RCLCPP_INFO(m_node->get_logger(), "-----------------------------");
    }

    inline bool Accumulator::readParameters()
    {
        if (!m_node->get_parameter("accumulation_time", m_params.accumulation_time))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter accumulation_time");
            return false;
        }
        if (!m_node->get_parameter("accumulation_number", m_params.accumulation_number))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter accumulation_number");
            return false;
        }
        if (!m_node->get_parameter("synchronous_decay", m_params.synchronous_decay))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter synchronous_decay");
            return false;
        }
        if (!m_node->get_parameter("min_potential", m_params.min_potential))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter min_potential");
            return false;
        }
        if (!m_node->get_parameter("max_potential", m_params.max_potential))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter max_potential");
            return false;
        }
        if (!m_node->get_parameter("neutral_potential", m_params.neutral_potential))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter neutral_potential");
            return false;
        }
        if (!m_node->get_parameter("event_contribution", m_params.event_contribution))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter event_contribution");
            return false;
        }
        if (!m_node->get_parameter("rectify_polarity", m_params.rectify_polarity))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter rectify_polarity");
            return false;
        }
        if (!m_node->get_parameter("decay_param", m_params.decay_param))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter decay_param");
            return false;
        }
        if (!m_node->get_parameter("slice_method", m_params.slice_method))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter slice_method");
            return false;
        }
        if (!m_node->get_parameter("decay_function", m_params.decay_function))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter decay_function");
            return false;
        }
        if (!m_node->get_parameter("accumulation_mode", m_params.accumulation_mode))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter accumulation_mode");
            return false;
        }
        if (!m_node->get_parameter("enable_decay", m_params.enable_decay))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter enable_decay");
            return false;
        }
        if (!m_node->get_parameter("decay_edge", m_params.decay_edge))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to read parameter decay_edge");
            return false;
        }
        return true;
    }

    rcl_interfaces::msg::SetParametersResult Accumulator::paramsCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "accumulation_time")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.accumulation_time = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "accumulation_time must be an integer";
                }
            }
            else if (param.get_name() == "accumulation_number")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.accumulation_number = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "accumulation_number must be an integer";
                }
            }
            else if (param.get_name() == "synchronous_decay")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    m_params.synchronous_decay = param.as_bool();
                }
                else
                {
                    result.successful = false;
                    result.reason = "synchronous_decay must be a boolean";
                }
            }
            else if (param.get_name() == "min_potential")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    m_params.min_potential = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "min_potential must be a double";
                }
            }
            else if (param.get_name() == "max_potential")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    m_params.max_potential = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "max_potential must be a double";
                }
            }
            else if (param.get_name() == "neutral_potential")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    m_params.neutral_potential = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "neutral_potential must be a double";
                }
            }
            else if (param.get_name() == "event_contribution")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    m_params.event_contribution = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "event_contribution must be a double";
                }
            }
            else if (param.get_name() == "rectify_polarity")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    m_params.rectify_polarity = param.as_bool();
                }
                else
                {
                    result.successful = false;
                    result.reason = "rectify_polarity must be a boolean";
                }
            }
            else if (param.get_name() == "decay_param")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    m_params.decay_param = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "decay_param must be a double";
                }
            }
            else if (param.get_name() == "slice_method")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.slice_method = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "slice_method must be an integer";
                }
            }
            else if (param.get_name() == "decay_function")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    m_params.decay_function = param.as_int();
                }
                else
                {
                    result.successful = false;
                    result.reason = "decay_function must be an integer";
                }
            }
            else if (param.get_name() == "accumulation_mode")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
                {
                    m_params.accumulation_mode = param.as_string();
                }
                else
                {
                    result.successful = false;
                    result.reason = "accumulation_mode must be a string";
                }
            }
            else if (param.get_name() == "enable_decay")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    m_params.enable_decay = param.as_bool();
                }
                else
                {
                    result.successful = false;
                    result.reason = "enable_decay must be a boolean";
                }
            }
            else if (param.get_name() == "decay_edge")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    m_params.decay_edge = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "decay_edge must be a double";
                }
            }
            else
            {
                result.successful = false;
                result.reason = "Unknown parameter";
            }
            updateConfiguration();
        }
        return result;
    }

}  // namespace dv_ros2_accumulation