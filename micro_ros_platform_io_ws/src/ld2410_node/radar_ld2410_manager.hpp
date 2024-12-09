#ifndef RADAR_LD2410_MANAGER_HPP_
#define RADAR_LD2410_MANAGER_HPP_

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <memory>
#include <map>

#include <ld2410_interface/msg/ld2410_target_data_frame.h>
#include <ld2410_interface/msg/ld2410_target_data_frame_array.h>
#include "ld2410_radar.hpp"
#include "uartconfig.hpp"
#include "wificonfig.hpp"
#include "rgb_led.hpp"

/**
 * @brief The state of the radar manager.
 */
enum class RadarManagerState : uint8_t
{
    WAITING_FOR_AGENT,
    CREATE_ROS_NODE,
    RUNNING_ROS_NODE, /*<-- Runs internal switch case / statemachine of micro_ros (i.e. check all handlers and execute them)*/
    DESTROY_ROS_NODE
};


/**
 * @brief Manager class for handling multiple LD2410 radar sensors.
 *
 * This class manages multiple LD2410 radar sensors, initializes them, reads data,
 * and publishes the data using micro-ROS.
 *
 * @tparam N_RADAR_SENSORS The number of radar sensors to manage.
 */
template <size_t N_RADAR_SENSORS> 
class RadarLd2410Manager
{
    public:
        #ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
        /**
         * @brief Construct a new RadarLd2410Manager object using UART configuration.
         *
         * @param node_name The name of the ROS node.
         * @param radar_publish_topic_name The name of the ROS topic to publish the radar data.
         * @param radar_configs An array of UART configurations for the radar sensors.
         * @param ros_serial_config The UART configuration for ROS serial communication.
         * @param device_id The device ID. This ID is used to identify which device or manager
         *                  the sensor is attached to, allowing a receiver to know from who the radar data is coming.
         */
        RadarLd2410Manager(const std::string& node_name, const std::string& radar_publish_topic_name, const std::array<UartConfig, N_RADAR_SENSORS>& radar_configs, 
                           const UartConfig& ros_serial_config, uint8_t device_id, uint8_t led_pin);
        
        #elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
        /**
         * @brief Construct a new RadarLd2410Manager object using WiFi configuration.
         *
         * @param node_name The name of the ROS node.
         * @param radar_publish_topic_name The name of the ROS topic to publish the radar data.
         * @param radar_configs An array of UART configurations for the radar sensors.
         * @param wifi_config The WiFi configuration for ROS communication.
         * @param device_id The device ID. This ID is used to identify which device or manager
         *                  the sensor is attached to, allowing a receiver to know from who the radar data is coming.
         */
        RadarLd2410Manager(const std::string& node_name, const std::string& radar_publish_topic_name, const std::array<UartConfig, N_RADAR_SENSORS>& radar_configs,
                           const WifiConfig& wifi_config, uint8_t device_id, uint8_t led_pin);
        #endif

        /**
         * @brief Destroy the RadarLd2410Manager object.
         */
        ~RadarLd2410Manager();

        /**
         * @brief Check if the ROS-agent is available. Pings once with a timeout of 1 second and returns the result.
         *
         * @return true if the agent is available, false otherwise.
         */
        bool isAgentAvailable();

        /**
         * @brief Initialize micro-ROS node.
         *
         * @return true if initialization was successful, false otherwise.
         */
        bool initMicroRos();

        /**
         * @brief Destroy micro-ROS node.
         */
        void destroyMicroRos();

        /**
         * @brief Spin the micro-ROS node one iteration with a timeout.
         *
         * @param timeout_ms The timeout in milliseconds. The function will return after the timeout, when no data is available to handle. Otherwise it will return directly after a 'handle'.
         * 
         * @return true if the node was spun successfully, false otherwise.
         */
        bool spinSome(uint32_t timeout_ms);

        /**
         * @brief Update the state machine. Run one iteration of the state machine.
         */
        void updateStateMachine();

    private:    
        /**
         * @brief Initialize the radar sensors with the given configurations.
         *
         * @param radar_configs An array of UART configurations for the radar sensors.
         * 
         * @note Used in the constructor.
         */
        void initializeRadars(const std::array<UartConfig, N_RADAR_SENSORS>& radar_configs);
            
        /**
         * @brief Initialize common parts of the radar manager. (There are two constructors, so this function is used to avoid code duplication.)
         * @note Used in the constructor.
         */
        void initCommonParts();

        /**
         * @brief Collect and publish radar data.
         * @note Could even publish if no data is available. (i.e. publish empty data)
         */
        void collectAndPublishRadarData();

        #ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
        std::unique_ptr<HardwareSerial> ros_serial_;
        #endif

        std::string node_name_;
        std::string radar_publish_topic_name_;
        uint8_t device_id_;
        RGBLed state_led_visualizer_;
        RadarManagerState current_state_;

        //Radar sensors
        std::array<Ld2410Radar,N_RADAR_SENSORS> sensors_;
        std::array<std::unique_ptr<HardwareSerial>,N_RADAR_SENSORS> serials_;

        //Node handles
        rcl_allocator_t allocator_;
        rclc_support_t support_;
        rcl_node_t node_;
        rclc_executor_t executor_;

        //Publish handles
        rcl_publisher_t target_frame_array_publisher_;
        rcl_timer_t publish_target_frame_array_timer_;
        ld2410_interface__msg__LD2410TargetDataFrameArray target_frame_array_msg_;

        static std::map<rcl_timer_t*, RadarLd2410Manager*> radar_manager_instance_map_;
};

#include "radar_ld2410_manager.tpp"

#endif // RADAR_LD2410_MANAGER_HPP