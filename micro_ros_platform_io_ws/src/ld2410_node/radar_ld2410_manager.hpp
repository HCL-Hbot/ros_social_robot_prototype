#ifndef RADAR_LD2410_MANAGER_HPP_
#define RADAR_LD2410_MANAGER_HPP_

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <memory>

#include <ld2410_interface/msg/ld2410_target_data_frame.h>
#include <ld2410_interface/msg/ld2410_target_data_frame_array.h>
#include "ld2410_radar.hpp"
#include "uartconfig.hpp"
#include "wificonfig.hpp"

template <size_t N_RADAR_SENSORS>  // N: aantal sensoren
class RadarLd2410Manager
{
    public:

        #ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
        /**
         * @brief Construct a new Radar Ld2410 Manager object
         */
        RadarLd2410Manager(const std::string& node_name, const UartConfig& ros_serial_config);
        
        #elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
        /**
         * @brief Construct a new Radar Ld 2 4 1 0 Manager object
         * 
         * @param node_name 
         * @param wifi_config 
         */
        RadarLd2410Manager(const std::string& node_name, WifiConfig& wifi_config);
        #endif

        /**
         * @brief Destroy the Radar Ld2410 Lld object
         * 
         */
        ~RadarLd2410Manager();

        void initializeRadars(const std::array<UartConfig, N_RADAR_SENSORS>& radar_configs);
        
        //void addRadars(const std::array<Ld2410Radar&, N_RADAR_SENSORS>& radars);

        void spin();
    private:

        void init_micro_ros(const std::string& node_name);

        std::unique_ptr<HardwareSerial> ros_serial_;

        //Node handles
        rcl_allocator_t allocator_;
        rclc_support_t support_;
        rcl_node_t node_;

        //Publish handles
        rcl_publisher_t test_;
        rcl_timer_t publish_timer_;
        //Custom message here.

        //Subscribe handles
        //rcl_subscription_t max_scan_range_sub_;
        //msg
        ld2410_interface__msg__LD2410TargetDataFrameArray target_frame_array_msg_;

        //Radar sensors
        std::array<Ld2410Radar,N_RADAR_SENSORS> sensors_;

        //pointer is needed, because hardwareserial does not have a default constructor.
        std::array<std::unique_ptr<HardwareSerial>,N_RADAR_SENSORS> serials_;
};

#include "radar_ld2410_manager.tpp"

#endif // RADAR_LD2410_MANAGER_HPP