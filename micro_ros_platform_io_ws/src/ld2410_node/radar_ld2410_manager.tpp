#ifndef RADAR_LD2410_MANAGER_TPP
#define RADAR_LD2410_MANAGER_TPP

#include "radar_ld2410_manager.hpp"


#define RC_RETURN_FALSE_ON_FAIL(fn) { const rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)) \
    { \
        Serial.print("Failed to init micro-ROS: "); \
        Serial.println(temp_rc); \
        Serial.print("On line: "); \
        Serial.println(__LINE__); \
        return false; \
    } \
    };//{return false;}}
    
// RCCHECK and RCSOFTCHECK Macros
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP.restart();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP.restart();}} 

template <unsigned int N_RADAR_SENSORS>
std::map<rcl_timer_t*, RadarLd2410Manager<N_RADAR_SENSORS>*> RadarLd2410Manager<N_RADAR_SENSORS>::radar_manager_instance_map_;

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
template <size_t N_RADAR_SENSORS>
RadarLd2410Manager<N_RADAR_SENSORS>::RadarLd2410Manager(const std::string& node_name, const std::string& radar_publish_topic_name, const UartConfig& ros_serial_config, uint8_t device_id)
: node_name_(node_name)
, radar_publish_topic_name_(radar_publish_topic_name)
, device_id_(device_id)
, most_recent_publish_failed_(false)
, allocator_(rcutils_get_default_allocator())
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

    
    target_frame_array_msg_.device_id = device_id_;
    target_frame_array_msg_.sensors.capacity = N_RADAR_SENSORS;
    target_frame_array_msg_.sensors.size = 0;
    target_frame_array_msg_.sensors.data = (ld2410_interface__msg__LD2410TargetDataFrame*) allocator_.allocate(target_frame_array_msg_.sensors.capacity * sizeof(ld2410_interface__msg__LD2410TargetDataFrame), allocator_.state);

    //map class timer with class instance. This way we can execute our callback function with a lambda. ROS C API does not have binding for callback function for non-static member functions.
    radar_manager_instance_map_[&publish_target_frame_array_timer_] = this;
}

#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
template <size_t N_RADAR_SENSORS>
RadarLd2410Manager<N_RADAR_SENSORS>::RadarLd2410Manager(const std::string& node_name, const std::string& radar_publish_topic_name, WifiConfig& wifi_config, uint8_t device_id)
: node_name_(node_name)
, radar_publish_topic_name_(radar_publish_topic_name)
, device_id_(device_id)
, most_recent_publish_failed_(false)
, allocator_(rcutils_get_default_allocator())

{
    IPAddress agent_ip;
    agent_ip.fromString(wifi_config.ip_.c_str());

    //Casting is safe in this case, implementation of wifi_transport won't modify the string. (Wifi library, which micro ros uses, casts it back to const char*)
    set_microros_wifi_transports(const_cast<char*>(wifi_config.ssid_.c_str()), const_cast<char*>(wifi_config.password_.c_str()), agent_ip, wifi_config.port_);

    target_frame_array_msg_.device_id = device_id_;
    target_frame_array_msg_.sensors.capacity = N_RADAR_SENSORS;
    target_frame_array_msg_.sensors.size = 0;
    target_frame_array_msg_.sensors.data = (ld2410_interface__msg__LD2410TargetDataFrame*) allocator_.allocate(target_frame_array_msg_.sensors.capacity * sizeof(ld2410_interface__msg__LD2410TargetDataFrame), allocator_.state);
    
    //map class timer with class instance. This way we can execute our callback function with a lambda. ROS C API does not have binding for callback function for non-static member functions.
    radar_manager_instance_map_[&publish_target_frame_array_timer_] = this;
}
#endif

template <size_t N_RADAR_SENSORS>
RadarLd2410Manager<N_RADAR_SENSORS>::~RadarLd2410Manager()
{
    radar_manager_instance_map_.erase(&publish_target_frame_array_timer_);
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::initializeRadars(const std::array<UartConfig, N_RADAR_SENSORS>& radar_configs)
{
    for(size_t i = 0; i<N_RADAR_SENSORS; ++i)
    {
        const UartConfig& radar_config = radar_configs[i];
        
        serials_[i].reset(new HardwareSerial(radar_config.uart_num_));
        serials_[i]->begin(radar_config.baudrate_, SERIAL_8N1, radar_config.rx_pin_, radar_config.tx_pin_);
        
        sensors_[i].begin(*(serials_[i]));
    }
}


template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::spin()
{
    Serial.println("Radar node begin spin loop");

    rclc_executor_spin(&executor_);
    Serial.println("Radar node exited spin loop");
    // pinMode(LED_BUILTIN, OUTPUT);
    // digitalWrite(LED_BUILTIN, HIGH);
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::spinSome(uint64_t timeout_ns)
{
    rclc_executor_spin_some(&executor_, timeout_ns);
}

template <size_t N_RADAR_SENSORS>
bool RadarLd2410Manager<N_RADAR_SENSORS>::mostRecentpublishFailed()
{
    return most_recent_publish_failed_;
}


template <size_t N_RADAR_SENSORS>
bool RadarLd2410Manager<N_RADAR_SENSORS>::isAgentAvialable()
{
    auto r = rmw_uros_ping_agent(1000,1);
    Serial.print("Ping agent: ");
    Serial.println(r);
    return r == RMW_RET_OK;
};

template <size_t N_RADAR_SENSORS>
bool RadarLd2410Manager<N_RADAR_SENSORS>::initMicroRos()
{   
    RC_RETURN_FALSE_ON_FAIL(rclc_support_init(&support_, 0, NULL, &allocator_));
    RC_RETURN_FALSE_ON_FAIL(rclc_node_init_default(&node_, node_name_.c_str(), "", &support_));

    //publisher radar
    RC_RETURN_FALSE_ON_FAIL(rclc_publisher_init_default(
         &target_frame_array_publisher_,
         &node_,
         ROSIDL_GET_MSG_TYPE_SUPPORT(ld2410_interface, msg, LD2410TargetDataFrameArray),
         radar_publish_topic_name_.c_str()));

    RC_RETURN_FALSE_ON_FAIL(rclc_timer_init_default2(
        &publish_target_frame_array_timer_,
        &support_,
        RCL_MS_TO_NS(1000),
        [](rcl_timer_t* timer, int64_t last_call_time)
        {
            RCLC_UNUSED(last_call_time);
            if(timer == nullptr) return;

            auto it = radar_manager_instance_map_.find(timer);
            if (it != radar_manager_instance_map_.end())
            {
                it->second->publishDetectedRegions();
            }    
        },
        true));

    RC_RETURN_FALSE_ON_FAIL(rclc_executor_init(&executor_, &support_.context, 1, &allocator_));

    RC_RETURN_FALSE_ON_FAIL(rclc_executor_add_timer(&executor_, &publish_target_frame_array_timer_));

    most_recent_publish_failed_ = false;

    return true; //return true if we pass all the checks above.
}

template <size_t N_RADAR_SENSORS>
bool RadarLd2410Manager<N_RADAR_SENSORS>::clean()
{
    rcl_ret_t ret;
    rcl_ret_t ret_total;
    //first destroy entities owned by the node.

    ret = rclc_executor_fini(&executor_);
    ret_total+=ret;
    Serial.print("Executor fini: ");
    Serial.println(ret);
    
    ret = rcl_publisher_fini(&target_frame_array_publisher_, &node_);
    ret_total+=ret;
    Serial.print("Publisher fini: ");
    Serial.println(ret);

    ret = rcl_timer_fini(&publish_target_frame_array_timer_);
    ret_total+=ret;
    Serial.print("Timer fini: ");
    Serial.println(ret);
    //lasty destroy the node.

    ret = rcl_node_fini(&node_);
    ret_total+=ret;
    Serial.print("Node fini: ");
    Serial.println(ret);

    ret = rclc_support_fini(&support_);
    ret_total+=ret;
    Serial.print("Support fini: ");
    Serial.println(ret);

    return ret_total == RCL_RET_OK;
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::publishDetectedRegions()
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

    rcl_ret_t status = rcl_publish(&target_frame_array_publisher_, &target_frame_array_msg_, NULL);
   
    if (status != RCL_RET_OK)
    {
        most_recent_publish_failed_ = true;
    }
    else
    {
        most_recent_publish_failed_ = false;
    }
 
}


#endif // RADAR_LD2410_MANAGER_TPP
