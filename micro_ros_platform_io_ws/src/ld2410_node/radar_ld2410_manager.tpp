#ifndef RADAR_LD2410_MANAGER_TPP
#define RADAR_LD2410_MANAGER_TPP

#include "radar_ld2410_manager.hpp"

void error_loop()
{
    while (1)
    {
        delay(100);
    }
}
// RCCHECK and RCSOFTCHECK Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} 


#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
template <size_t N_RADAR_SENSORS>
RadarLd2410Manager<N_RADAR_SENSORS>::RadarLd2410Manager(const std::string& node_name, const UartConfig& ros_serial_config)
{
    ros_serial_.reset(new HardwareSerial(ros_serial_config.uart_num_));

    //Use the internal UART_to_USB bridge (only possible with UART 0) for ros serial communciation.
    if(ros_serial_config.uart_num_ == 0 && ros_serial_config.rx_pin_ == 0 && ros_serial_config.tx_pin_ == 0) 
    {
        ros_serial_->begin(ros_serial_config.baudrate_);
    }
    else
    {
        ros_serial_->begin(ros_serial_config.baudrate_, SERIAL_8N1, ros_serial_config.rx_pin_, ros_serial_config.tx_pin_);
    }
    set_microros_serial_transports(*ros_serial_);
    delay(2000); //give some time to init.
}

#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
template <size_t N_RADAR_SENSORS>
RadarLd2410Manager<N_RADAR_SENSORS>::RadarLd2410Manager(const std::string& node_name,  WifiConfig& wifi_config)
{
    IPAddress agent_ip;
    agent_ip.fromString(wifi_config.ip_.c_str());

    //Casting is safe in this case, implementation of wifi_transport won't modify the string. (Wifi library, which micro ros uses, casts it back to const char*)
    set_microros_wifi_transports(const_cast<char*>(wifi_config.ssid_.c_str()), const_cast<char*>(wifi_config.password_.c_str()), agent_ip, wifi_config.port_);
}
#endif

template <size_t N_RADAR_SENSORS>
RadarLd2410Manager<N_RADAR_SENSORS>::~RadarLd2410Manager()
{
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::initializeRadars(const std::array<UartConfig, N_RADAR_SENSORS>& radar_configs)
{
    for(size_t i = 0; i<N_RADAR_SENSORS; ++i)
    {
        const UartConfig& radar_config = radar_configs[i];
        
        serials_[i].reset(new HardwareSerial(radar_config.uart_num_));
        serials_[i]->begin(radar_config.baudrate_, SERIAL_8N1, radar_config.rx_pin_, radar_config.tx_pin_);
        
        //std::unique_ptr<HardwareSerial> serial(new HardwareSerial(radar_config.uart_num_));

        //serial->begin(radar_config.baudrate_, SERIAL_8N1, radar_config.rx_pin_, radar_config.tx_pin_);

        //serials_[i].swap(serial);
        
        sensors_[i].begin(*(serials_[i]));
    }
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::spin()
{
    target_frame_array_msg_.sensors.size = 0;

    for (size_t i = 0; i < N_RADAR_SENSORS; i++)
    {
        ld2410_interface__msg__LD2410TargetDataFrame temp_target_frame;
        Ld2410Radar& sensor = sensors_[i];

        if (sensor.read())
        {
            const TargetFrameData& fd = sensor.getCurrentTargetFrame();
            temp_target_frame.rader_id = i;
            temp_target_frame.target_state = fd.target_state_;
            temp_target_frame.movement_distance = fd.movement_distance_;
            temp_target_frame.movement_energy = fd.movement_energy_;
            temp_target_frame.stationaty_distance = fd.stationaty_distance_;
            temp_target_frame.stationaty_energy = fd.stationaty_energy_;
            temp_target_frame.detection_distance = fd.detection_distance_;
            
            target_frame_array_msg_.sensors.data[target_frame_array_msg_.sensors.size] = temp_target_frame;
            target_frame_array_msg_.sensors.size++;
        }
    }
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::init_micro_ros(const std::string& node_name)
{
    allocator_ = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));
    RCCHECK(rclc_node_init_default(&node_, node_name.c_str(), "", &support_));

    target_frame_array_msg_.device_id = 0;
    target_frame_array_msg_.sensors.capacity = N_RADAR_SENSORS;
    target_frame_array_msg_.sensors.size = 0;
    target_frame_array_msg_.sensors.data = malloc(N_RADAR_SENSORS * sizeof(ld2410_interface__msg__LD2410TargetDataFrame));


    //led_response_msg.message.data = (char*) allocator.allocate(led_response_msg.message.capacity * sizeof(char), allocator.state);
    //target_frame_array_msg_.sensors.data = (char*) allocator.allocate(target_frame_array_msg_.message.capacity * sizeof(char), allocator.state);
    // // Initialize publisher
    // RCCHECK(rclc_publisher_init_default(
    //     &led_status_publisher,
    //     &node_,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    //     "led_status"));
}

#endif // RADAR_LD2410_MANAGER_TPP
