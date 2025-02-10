#include "radar_presence_hld.hpp"
#include "ld2410_interface/msg/ld2410_target_data_frame.hpp"

constexpr const char* DEFAULT_NODE_NAME = "radar_presence_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_RADAR_PRESENCE_LL = "ld2410_target_frames";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_RADAR_PRESENCE_HL = "radar_presence";
constexpr const uint16_t DISTANCE_TRESHOLD_CM = 200;

RadarPresenceHLD::RadarPresenceHLD() :
rclcpp::Node(DEFAULT_NODE_NAME),
radar_presence_subscriber_(create_subscription<ld2410_interface::msg::LD2410TargetDataFrameArray>(
        DEFAULT_TOPIC_NAME_SUB_RADAR_PRESENCE_LL, 10, std::bind(&RadarPresenceHLD::radarPresenceCallback, this, std::placeholders::_1))),
radar_presence_publisher_(this->create_publisher<radar_presence_hld::msg::PresenceDetection>(DEFAULT_TOPIC_NAME_PUB_RADAR_PRESENCE_HL, 10)),
current_radar_presence_msg_()
{
   current_radar_presence_msg_.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_STANDING;
   current_radar_presence_msg_.presence_state = radar_presence_hld::msg::PresenceDetection::TARGET_OUT_OF_RANGE;
}

RadarPresenceHLD::~RadarPresenceHLD()
{
}

void RadarPresenceHLD::radarPresenceCallback(const ld2410_interface::msg::LD2410TargetDataFrameArray::SharedPtr msg)
{
    radar_presence_hld::msg::PresenceDetection new_radar_presence_msg;
    new_radar_presence_msg.target_state = current_radar_presence_msg_.target_state;
    new_radar_presence_msg.presence_state = current_radar_presence_msg_.presence_state;

    for (auto& radar_sensor : msg->sensors)
    {
        if(radar_sensor.target_state == ld2410_interface::msg::LD2410TargetDataFrame::NO_TARGET)
        {
            continue; //skip this sensor
        }
        else
        {
            uint16_t target_distance_cm = getDistanceFromSensor(radar_sensor);
            
            if(new_radar_presence_msg.presence_state == radar_presence_hld::msg::PresenceDetection::TARGET_OUT_OF_RANGE &&
               target_distance_cm <= DISTANCE_TRESHOLD_CM)
            {
                new_radar_presence_msg.presence_state = radar_presence_hld::msg::PresenceDetection::TARGET_IN_RANGE;

                if(radar_sensor.target_state == ld2410_interface::msg::LD2410TargetDataFrame::STATIONARY_ONLY)
                {
                    new_radar_presence_msg.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_STANDING;
                }
                else
                {
                    new_radar_presence_msg.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_MOVING;
                }
                break; //no need to check further we have a target!
            }
            else if(new_radar_presence_msg.presence_state == radar_presence_hld::msg::PresenceDetection::TARGET_IN_RANGE &&
               target_distance_cm > DISTANCE_TRESHOLD_CM)
            {
                new_radar_presence_msg.presence_state = radar_presence_hld::msg::PresenceDetection::TARGET_OUT_OF_RANGE;

                if(radar_sensor.target_state == ld2410_interface::msg::LD2410TargetDataFrame::STATIONARY_ONLY)
                {
                    new_radar_presence_msg.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_STANDING;
                }
                else
                {
                    new_radar_presence_msg.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_MOVING;
                }
            }
        }
    }

    //publish the new state if it has changed
    if(current_radar_presence_msg_.presence_state != new_radar_presence_msg.presence_state || current_radar_presence_msg_.target_state != new_radar_presence_msg.target_state)
    {   
        current_radar_presence_msg_.target_state = new_radar_presence_msg.target_state;
        current_radar_presence_msg_.presence_state = new_radar_presence_msg.presence_state;
        radar_presence_publisher_->publish(current_radar_presence_msg_);
    }
}

uint16_t RadarPresenceHLD::getDistanceFromSensor(const ld2410_interface::msg::LD2410TargetDataFrame& sensor)
{
    uint16_t target_distance_cm = 0;

    switch(sensor.target_state)
    {
        case ld2410_interface::msg::LD2410TargetDataFrame::MOVING_ONLY:
            target_distance_cm = sensor.movement_distance;
            break;
        case ld2410_interface::msg::LD2410TargetDataFrame::STATIONARY_ONLY:
            target_distance_cm = sensor.stationaty_distance;
            break;
        case ld2410_interface::msg::LD2410TargetDataFrame::MOVING_AND_STATIONARY: 
            //use the detection distance because frame could be either moving or stationary (not documented well in the datasheet)
            target_distance_cm = sensor.detection_distance;
            break;
        default:
            break;
    }
    return target_distance_cm;
}
