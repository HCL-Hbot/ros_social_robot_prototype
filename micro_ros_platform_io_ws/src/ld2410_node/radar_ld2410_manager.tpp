#ifndef RADAR_LD2410_MANAGER_TPP
#define RADAR_LD2410_MANAGER_TPP

#include "radar_ld2410_manager.hpp"


#define RC_RETURN_FALSE_ON_FAIL(fn) \
        { \
            const rcl_ret_t temp_rc = fn; \
            if((temp_rc != RCL_RET_OK)) \
            { \
                return false; \
            } \
        };
    
template <unsigned int N_RADAR_SENSORS>
std::map<rcl_timer_t*, RadarLd2410Manager<N_RADAR_SENSORS>*> RadarLd2410Manager<N_RADAR_SENSORS>::radar_manager_instance_map_;

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
template <size_t N_RADAR_SENSORS>
RadarLd2410Manager<N_RADAR_SENSORS>::RadarLd2410Manager(const std::string& node_name, const std::string& radar_publish_topic_name, const std::array<UartConfig, N_RADAR_SENSORS>& radar_configs,
                                                        const UartConfig& ros_serial_config, uint8_t device_id, uint8_t led_pin)
: node_name_(node_name)
, radar_publish_topic_name_(radar_publish_topic_name)
, device_id_(device_id)
, state_led_visualizer_(led_pin)
, current_state_(RadarManagerState::WAITING_FOR_AGENT)
, sensors_()
, serials_()
, allocator_(rcutils_get_default_allocator())
, node_(rcl_get_zero_initialized_node())
, target_frame_array_publisher_(rcl_get_zero_initialized_publisher())
, publish_target_frame_array_timer_(rcl_get_zero_initialized_timer())
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

    initializeRadars(radar_configs);
    initCommonParts();
}

#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
template <size_t N_RADAR_SENSORS>
RadarLd2410Manager<N_RADAR_SENSORS>::RadarLd2410Manager(const std::string& node_name, const std::string& radar_publish_topic_name, const std::array<UartConfig, N_RADAR_SENSORS>& radar_configs,
                                                        const WifiConfig& wifi_config, uint8_t device_id, uint8_t led_pin)
: node_name_(node_name)
, radar_publish_topic_name_(radar_publish_topic_name)
, device_id_(device_id)
, state_led_visualizer_(led_pin)
, current_state_(RadarManagerState::WAITING_FOR_AGENT)
, sensors_()
, serials_()
, allocator_(rcutils_get_default_allocator())
, node_(rcl_get_zero_initialized_node())
, target_frame_array_publisher_(rcl_get_zero_initialized_publisher())
, publish_target_frame_array_timer_(rcl_get_zero_initialized_timer())
{
    IPAddress agent_ip;
    agent_ip.fromString(wifi_config.ip_.c_str());

    //Casting is safe in this case, implementation of wifi_transport won't modify the string. (Wifi library, which micro ros uses, casts it back to const char*)
    set_microros_wifi_transports(const_cast<char*>(wifi_config.ssid_.c_str()), const_cast<char*>(wifi_config.password_.c_str()), agent_ip, wifi_config.port_);
    
    initializeRadars(radar_configs);
    initCommonParts();
}
#endif

template <size_t N_RADAR_SENSORS>
RadarLd2410Manager<N_RADAR_SENSORS>::~RadarLd2410Manager()
{
    radar_manager_instance_map_.erase(&publish_target_frame_array_timer_);
    destroyMicroRos();
}

template <size_t N_RADAR_SENSORS>
bool RadarLd2410Manager<N_RADAR_SENSORS>::isAgentAvailable()
{
    return rmw_uros_ping_agent(1000,1) == RMW_RET_OK;
};

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
                it->second->collectAndPublishRadarData();
            }    
        },
        true));

    RC_RETURN_FALSE_ON_FAIL(rclc_executor_init(&executor_, &support_.context, 1, &allocator_));

    RC_RETURN_FALSE_ON_FAIL(rclc_executor_add_timer(&executor_, &publish_target_frame_array_timer_));

    return true; //return true if we pass all the checks above.
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::destroyMicroRos()
{
    rcl_ret_t ret;

    //first destroy entities owned by the node.
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support_.context);
    ret = rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    ret = rcl_publisher_fini(&target_frame_array_publisher_, &node_);
    ret = rcl_timer_fini(&publish_target_frame_array_timer_);
    ret = rclc_executor_fini(&executor_);
    
    //lasty destroy the node.
    ret = rcl_node_fini(&node_);
    ret = rclc_support_fini(&support_);

    RCLC_UNUSED(ret); //suppress warning, because we are not using the result.
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::updateStateMachine()
{
    switch (current_state_)
    {
        case RadarManagerState::WAITING_FOR_AGENT:
            if(isAgentAvailable())
            {
                current_state_ = RadarManagerState::CREATE_ROS_NODE;
                state_led_visualizer_.setColor(0, 255, 255); //cyan 
                state_led_visualizer_.show();
            }
            break;

        case RadarManagerState::CREATE_ROS_NODE:
            if(initMicroRos())
            {
                current_state_ = RadarManagerState::RUNNING_ROS_NODE;
                state_led_visualizer_.setColor(0, 255, 0); //green 
                state_led_visualizer_.show();
            }
            else
            {
                current_state_ = RadarManagerState::DESTROY_ROS_NODE;
            }
            break;
        
        case RadarManagerState::RUNNING_ROS_NODE:
            rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1000));

            if(!isAgentAvailable())
            {
                current_state_ = RadarManagerState::DESTROY_ROS_NODE;
                state_led_visualizer_.setColor(255, 0, 0); //red 
                state_led_visualizer_.show();
            }
            break;
        
        case RadarManagerState::DESTROY_ROS_NODE:
            destroyMicroRos();
            current_state_ = RadarManagerState::WAITING_FOR_AGENT;
            state_led_visualizer_.setColor(0, 0, 255); //blue 
            state_led_visualizer_.show();
            break;

        default:
            break;
    }
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::initCommonParts()
{
    target_frame_array_msg_.device_id = device_id_;
    target_frame_array_msg_.sensors.capacity = N_RADAR_SENSORS;
    target_frame_array_msg_.sensors.size = 0;
    target_frame_array_msg_.sensors.data = (ld2410_interface__msg__LD2410TargetDataFrame*) allocator_.allocate(target_frame_array_msg_.sensors.capacity * sizeof(ld2410_interface__msg__LD2410TargetDataFrame), allocator_.state);
    
    //map class timer with class instance. This way we can execute our callback function with a lambda. ROS C API does not have binding for callback function for non-static member functions.
    radar_manager_instance_map_[&publish_target_frame_array_timer_] = this;

    state_led_visualizer_.setBrightness(10);
    state_led_visualizer_.setColor(0, 0, 255); //blue 
    state_led_visualizer_.show();
}

template <size_t N_RADAR_SENSORS>
void RadarLd2410Manager<N_RADAR_SENSORS>::collectAndPublishRadarData()
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

  
    rcl_ret_t publish_result = rcl_publish(&target_frame_array_publisher_, &target_frame_array_msg_, NULL);

    RCL_UNUSED(publish_result); //suppress warning, because we are not using the result.
}


#endif // RADAR_LD2410_MANAGER_TPP
