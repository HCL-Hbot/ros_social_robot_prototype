#ifndef RADAR_PRESENCE_HLD_INCLUDE_RADAR_PRESENCE_HLD_HPP_
#define RADAR_PRESENCE_HLD_INCLUDE_RADAR_PRESENCE_HLD_HPP_

#include <rclcpp/rclcpp.hpp>
#include "ld2410_interface/msg/ld2410_target_data_frame_array.hpp"
#include "radar_presence_hld/msg/presence_detection.hpp"
class RadarPresenceHLD : public rclcpp::Node
{
public:
    RadarPresenceHLD();
    virtual ~RadarPresenceHLD();

private:
    void radarPresenceCallback(const ld2410_interface::msg::LD2410TargetDataFrameArray::SharedPtr msg);
    uint16_t getDistanceFromSensor(const ld2410_interface::msg::LD2410TargetDataFrame& sensor);
    rclcpp::Subscription<ld2410_interface::msg::LD2410TargetDataFrameArray>::SharedPtr radar_presence_subscriber_;
    rclcpp::Publisher<radar_presence_hld::msg::PresenceDetection>::SharedPtr radar_presence_publisher_;
    radar_presence_hld::msg::PresenceDetection current_radar_presence_msg_;
};




#endif // RADAR_PRESENCE_HLD_INCLUDE_RADAR_PRESENCE_HLD_HPP_