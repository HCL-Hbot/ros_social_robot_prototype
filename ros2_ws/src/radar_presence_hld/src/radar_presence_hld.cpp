#include "radar_presence_hld.hpp"
#include "ld2410_interface/msg/ld2410_target_data_frame.hpp"

constexpr const char* DEFAULT_NODE_NAME = "radar_presence_hld_node";
constexpr const char* DEFAULT_TOPIC_NAME_SUB_RADAR_PRESENCE_LL = "ld2410_target_frames";
constexpr const char* DEFAULT_TOPIC_NAME_PUB_RADAR_PRESENCE_HL = "radar_presence";
constexpr const uint16_t DISTANCE_TRESHOLD_CM = 200;

RadarPresenceHLD::RadarPresenceHLD() :
rclcpp::Node(DEFAULT_NODE_NAME),
radar_lld_sub_(create_subscription<ld2410_interface::msg::LD2410TargetDataFrameArray>(
        DEFAULT_TOPIC_NAME_SUB_RADAR_PRESENCE_LL, 10, std::bind(&RadarPresenceHLD::radarPresenceCallback, this, std::placeholders::_1))),
radar_presence_pub_(this->create_publisher<radar_presence_hld::msg::PresenceDetection>(DEFAULT_TOPIC_NAME_PUB_RADAR_PRESENCE_HL, 10)),
current_radar_presence_msg_()
{
   current_radar_presence_msg_.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_STANDING;
   current_radar_presence_msg_.presence_state = radar_presence_hld::msg::PresenceDetection::TARGET_OUT_OF_RANGE;
}

RadarPresenceHLD::~RadarPresenceHLD()
{
}

void RadarPresenceHLD::radarPresenceCallback(const ld2410_interface::msg::LD2410TargetDataFrameArray::SharedPtr radar_presence_msg)
{
    // Search for the radar sensor with the smallest distance
    const ld2410_interface::msg::LD2410TargetDataFrame* smallest_distance_sensor = findSmallestDistanceSensor(radar_presence_msg);

    if (smallest_distance_sensor != nullptr)
    {
        // Translate the smallest radar_sensor to a PresenceDetection message
        auto new_radar_presence_msg = translateToPresenceDetection(*smallest_distance_sensor);

        bool is_presence_changed = isPresenceDetectionChanged(new_radar_presence_msg);
        if (is_presence_changed)
        {
            updateAndPublishPresenceDetection(new_radar_presence_msg);
        }
    }
}

const ld2410_interface::msg::LD2410TargetDataFrame* RadarPresenceHLD::findSmallestDistanceSensor(const ld2410_interface::msg::LD2410TargetDataFrameArray::SharedPtr radar_presence_msg)
{
    const ld2410_interface::msg::LD2410TargetDataFrame* smallest_distance_sensor = nullptr;
    uint16_t smallest_distance = std::numeric_limits<uint16_t>::max();

    for (const auto& radar_sensor : radar_presence_msg->sensors)
    {
        if (radar_sensor.target_state != ld2410_interface::msg::LD2410TargetDataFrame::NO_TARGET)
        {
            uint16_t target_distance_cm = getDistanceFromSensor(radar_sensor);
            if (target_distance_cm < smallest_distance)
            {
                smallest_distance = target_distance_cm;
                smallest_distance_sensor = &radar_sensor;
            }
        }
    }

    return smallest_distance_sensor;
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

radar_presence_hld::msg::PresenceDetection RadarPresenceHLD::translateToPresenceDetection(const ld2410_interface::msg::LD2410TargetDataFrame& radar_sensor)
{
    radar_presence_hld::msg::PresenceDetection new_radar_presence_msg;

    //This case should not be possible, because normally we filter this out before calling this function.
    //But for good practice we check for this scenario. PresenceDetecion should stay the same in this case.
    if (radar_sensor.target_state == ld2410_interface::msg::LD2410TargetDataFrame::NO_TARGET)
    {
        new_radar_presence_msg.presence_state = current_radar_presence_msg_.presence_state;
        new_radar_presence_msg.target_state = current_radar_presence_msg_.target_state;
    }
    else
    {
        uint16_t target_distance_cm = getDistanceFromSensor(radar_sensor);

        if (target_distance_cm <= DISTANCE_TRESHOLD_CM)
        {
            new_radar_presence_msg.presence_state = radar_presence_hld::msg::PresenceDetection::TARGET_IN_RANGE;

            if (radar_sensor.target_state == ld2410_interface::msg::LD2410TargetDataFrame::STATIONARY_ONLY)
            {
                new_radar_presence_msg.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_STANDING;
            }
            else
            {
                new_radar_presence_msg.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_MOVING;
            }
        }
        else
        {
            new_radar_presence_msg.presence_state = radar_presence_hld::msg::PresenceDetection::TARGET_OUT_OF_RANGE;

            if (radar_sensor.target_state == ld2410_interface::msg::LD2410TargetDataFrame::STATIONARY_ONLY)
            {
                new_radar_presence_msg.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_STANDING;
            }
            else
            {
                new_radar_presence_msg.target_state = radar_presence_hld::msg::PresenceDetection::TARGET_MOVING;
            }
        }
    }

    return new_radar_presence_msg;
}

bool RadarPresenceHLD::isPresenceDetectionChanged(const radar_presence_hld::msg::PresenceDetection& new_radar_presence_msg)
{
    return current_radar_presence_msg_.presence_state != new_radar_presence_msg.presence_state ||
           current_radar_presence_msg_.target_state != new_radar_presence_msg.target_state;
}

void RadarPresenceHLD::updateAndPublishPresenceDetection(const radar_presence_hld::msg::PresenceDetection& new_radar_presence_msg)
{
    current_radar_presence_msg_.target_state = new_radar_presence_msg.target_state;
    current_radar_presence_msg_.presence_state = new_radar_presence_msg.presence_state;
    radar_presence_pub_->publish(current_radar_presence_msg_);
}