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
         * @param ros_serial_config The UART configuration for ROS serial communication.
         * @param device_id The device ID. This ID is used to identify which device or manager
         *                  the sensor is attached to, allowing a receiver to know from who the radar data is coming.
         */
        RadarLd2410Manager(const std::string& node_name, const std::string& radar_publish_topic_name, const UartConfig& ros_serial_config, uint8_t device_id);
        
        #elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
        /**
         * @brief Construct a new RadarLd2410Manager object using WiFi configuration.
         *
         * @param node_name The name of the ROS node.
         * @param radar_publish_topic_name The name of the ROS topic to publish the radar data.
         * @param wifi_config The WiFi configuration for ROS communication.
         * @param device_id The device ID. This ID is used to identify which device or manager
         *                  the sensor is attached to, allowing a receiver to know from who the radar data is coming.
         */
        RadarLd2410Manager(const std::string& node_name, const std::string& radar_publish_topic_name, WifiConfig& wifi_config, uint8_t device_id);
        #endif

        /**
         * @brief Destroy the RadarLd2410Manager object.
         */
        ~RadarLd2410Manager();

        /**
         * @brief Check if the ROS-agent is available. Pings once and returns the result.
         *
         * @return true if the agent is available, false otherwise.
         */
        bool isAgentAvialable();

        bool initMicroRos();

        /**
         * @brief Initialize the radar sensors with the given configurations.
         *
         * @param radar_configs An array of UART configurations for the radar sensors.
         */
        void initializeRadars(const std::array<UartConfig, N_RADAR_SENSORS>& radar_configs);
        
        /**
         * @brief Spin the ROS node to process callbacks.
         * @note  Blocking function.
         */
        void spin();

        /**
         * @brief Spin the ROS node to process callbacks for a specified duration.
         *
         * @param timeout_ns The timeout duration in nanoseconds.
         */
        void spinSome(uint64_t timeout_ns);

       
        bool mostRecentpublishFailed();

        bool clean();
    private:

        

        void publishDetectedRegions();

        #ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
        std::unique_ptr<HardwareSerial> ros_serial_;
        #endif

        std::string node_name_;
        std::string radar_publish_topic_name_;
        uint8_t device_id_;

        bool most_recent_publish_failed_;

        //Node handles
        rcl_allocator_t allocator_;
        rclc_support_t support_;
        rcl_node_t node_;
        rclc_executor_t executor_;


        //Publish handles
        rcl_publisher_t target_frame_array_publisher_;
        rcl_timer_t publish_target_frame_array_timer_;
        ld2410_interface__msg__LD2410TargetDataFrameArray target_frame_array_msg_;

        //Radar sensors
        std::array<Ld2410Radar,N_RADAR_SENSORS> sensors_;

        //pointer is needed, because hardwareserial does not have a default constructor.
        std::array<std::unique_ptr<HardwareSerial>,N_RADAR_SENSORS> serials_;

        static std::map<rcl_timer_t*, RadarLd2410Manager*> radar_manager_instance_map_;

};

#include "radar_ld2410_manager.tpp"

#endif // RADAR_LD2410_MANAGER_HPP