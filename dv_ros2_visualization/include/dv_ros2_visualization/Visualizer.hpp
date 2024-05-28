#pragma once
// C++ System Headers
#include <boost/lockfree/spsc_queue.hpp>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// dv-processing Headers
#include <dv-processing/core/frame.hpp>
#include <dv-processing/processing.hpp>

// dv_ros2_msgs Headers
#include "dv_ros2_msgs/msg/event.hpp"
#include "dv_ros2_msgs/msg/event_array.hpp"
#include "dv_ros2_messaging/messaging.hpp"

namespace dv_ros2_visualization
{
    struct Params
    {
        /// @brief topic name of the image to be published
        std::string image_topic = "image";
        /// @brief frame rate of publishing the image
        double frame_rate = 60.0;
        /// @brief RGB values of the background color
        int16_t background_color_r;
        int16_t background_color_g;
        int16_t background_color_b;
        /// @brief RGB values of the positive event color
        int16_t positive_event_color_r;
        int16_t positive_event_color_g;
        int16_t positive_event_color_b;
        /// @brief RGB values of the negative event color
        int16_t negative_event_color_r;
        int16_t negative_event_color_g;
        int16_t negative_event_color_b;
    };

    class Visualizer : public rclcpp::Node
    {
        using rclcpp::Node::Node;
    public:
        /// @brief Default constructor
        /// @param t_node_name name of the node
        Visualizer(const std::string &t_node_name);

        /// @brief Shallow copy constructor
        /// @param source source object to copy from
        Visualizer(const Visualizer &source);

        /// @brief Destructor
        ~Visualizer();

        /// @brief Start the visualizer
        void start();

        /// @brief Stop the visualizer
        void stop();

        /// @brief Check if the visualizer is running
        /// @return true if the visualizer is running, false otherwise
        bool isRunning() const;

        /// @brief ROS2 parameter callback
        /// @param parameters vector of parameters
        /// @return SetParametersResult
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter> &parameters);
    private:
        /// @brief Parameter initialization
        inline void parameterInitilization() const;

        /// @brief Print parameters
        inline void parameterPrinter() const;

        /// @brief Reads the std library variables and ROS2 parameters
        /// @return true if all parameters are read successfully
        inline bool readParameters();

        /// @brief Update configuration for reconfiguration while running
        void updateConfiguration();

        /// @brief Event callback function for populating queue
        /// @param events EventArray message
        void eventCallback(dv_ros2_msgs::msg::EventArray::SharedPtr events);

        /// @brief Slicer callback function
        void slicerCallback(const dv::EventStore &events);
        
        /// @brief Visualization thread
        void visualize();

        /// @brief rclcpp node pointer
        rclcpp::Node::SharedPtr m_node;

        /// @brief Parameters
        Params m_params;

        // Thread realted
        std::atomic<bool> m_spin_thread = true;
        std::thread m_visualization_thread;

        /// @brief EventArray subscriber
        rclcpp::Subscription<dv_ros2_msgs::msg::EventArray>::SharedPtr m_events_subscriber;

        /// @brief Frame publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_frame_publisher;

        /// @brief Event queue
        boost::lockfree::spsc_queue<dv::EventStore> m_event_queue{100};

        /// @brief Slicer object
        std::unique_ptr<dv::EventStreamSlicer> m_slicer = nullptr;
        
        /// @brief Visualizer object
        std::unique_ptr<dv::visualization::EventVisualizer> m_visualizer = nullptr;

        /// @brief Job ID of the slicer, used to stop jobs running in the slicer
        std::optional<int> m_job_id;
    };
} // namespace dv_ros2_visualization