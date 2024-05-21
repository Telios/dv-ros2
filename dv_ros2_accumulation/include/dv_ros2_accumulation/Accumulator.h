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
#include <boost/lockfree/spsc_queue.hpp>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

// dv-processing Headers
#include <dv-processing/core/frame.hpp>

// dv_ros2_msgs Headers
#include "dv_ros2_msgs/msg/event.hpp"
#include "dv_ros2_msgs/msg/event_array.hpp"
#include "dv_ros2_messaging/messaging.hpp"

namespace dv_ros2_accumulation
{
    enum SliceMethod
    {
        TIME = 0,
        NUMBER = 1
    };


    struct Params
    {
        /// @brief Time in ms to accumulate events over [1,1000]
        int32_t accumulation_time = 33;
        /// @brief Number of events to accumulate for a frame [1000,10000000]
        int32_t accumulation_number = 100000;
        /// @brief Decay at frame generation time 
        bool synchronous_decay = false;
        /// @brief Value at which to clip the integration [0.0,1.0]
        double min_potential = 0.0;
        /// @brief Value at which to clip the integration [0.0,1.0]
        double max_potential = 1.0;
        /// @brief Value to which the decay tends over time [0.0,1.0]
        double neutral_potential = 0.0;
        /// @brief The contribution of a single event [0.0,1.0]
        double event_contribution = 0.15;
        /// @brief All events have positive contribution 
        bool rectify_polarity = false;
        /// @brief Slope for linear decay, tau for  exponential decay, time for step decay [0.0,1e+10]
        double decay_param = 1e+6;
        /// @brief Method to slice the accumulation [TIME, NUMBER]
        int slice_method = static_cast<int>(SliceMethod::TIME);
        /// @brief Decay function to use [NONE, LINEAR, EXPONENTIAL, STEP]
        int decay_function = static_cast<int>(dv::Accumulator::Decay::LINEAR);
        /// @brief Mode of the accumulation [EDGE, FRAME]
        std::string accumulation_mode = "FRAME";
        /// @brief Enable linear decay
        bool enable_decay = false;
        /// @brief Slope for linear decay, tau for  exponential decay, time for step decay [0.0, 1.0]
        double decay_edge = 0.1;
    };
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

        /// @brief Start the accumulation
        void start();

        /// @brief Stop accumulation
        void stop();

        /// @brief Check if the node is running
        /// @return true if the node is running
        bool isRunning() const;
    private:
        /// @brief Parameter initialization
        inline void parameterInitilization() const;

        /// @brief Print parameters
        inline void parameterPrinter() const;

        /// @brief Reads the std library variables and ROS2 parameters
        /// @return true if all parameters are read successfully
        inline bool readParameters();

        /// @brief Event callback function for populating queue
        /// @param events EventArray message
        void eventCallback(dv_ros2_msgs::msg::EventArray::SharedPtr events);

        /// @brief Accumulation thread
        void accumulate();

        void slicerCallback(const dv::EventStore &events);

        void updateConfiguration();

        /// @brief eclcpp node variable
        rclcpp::Node::SharedPtr m_node;

        /// @brief Params struct
        Params m_params;

        // Thread related
        std::atomic<bool> m_spin_thread = true;
        std::thread m_accumulation_thread;

        // EventArray subscriber
        rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr m_events_subscriber;

        // Frame publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_frame_publisher;

        boost::lockfree::spsc_queue<dv::EventStore> m_event_queue{100};

        std::unique_ptr<dv::Accumulator> m_accumulator = nullptr;
        std::unique_ptr<dv::PixelAccumulator> m_accumulator_edge = nullptr;


        std::unique_ptr<dv::EventStreamSlicer> m_slicer = nullptr;


    };
}  // namespace dv_ros2_accumulation