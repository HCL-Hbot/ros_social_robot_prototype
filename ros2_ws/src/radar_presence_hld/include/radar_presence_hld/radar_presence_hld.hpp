#ifndef RADAR_PRESENCE_HLD_INCLUDE_RADAR_PRESENCE_HLD_HPP_
#define RADAR_PRESENCE_HLD_INCLUDE_RADAR_PRESENCE_HLD_HPP_

#include "ld2410_interface/msg/ld2410_target_data_frame_array.hpp"
#include "interaction_controller/msg/presence_detection.hpp"

#include <rclcpp/rclcpp.hpp>

namespace radar_hld {

class RadarPresenceHLD : public rclcpp::Node
{
public:
    RadarPresenceHLD();
    virtual ~RadarPresenceHLD();

private:
    void radarPresenceCallback(const ld2410_interface::msg::LD2410TargetDataFrameArray::SharedPtr radar_presence_msg);
    
    const ld2410_interface::msg::LD2410TargetDataFrame* findSmallestDistanceSensor(const ld2410_interface::msg::LD2410TargetDataFrameArray::SharedPtr radar_presence_msg);

    uint16_t getDistanceFromSensor(const ld2410_interface::msg::LD2410TargetDataFrame& sensor);

    interaction_controller::msg::PresenceDetection translateToPresenceDetection(const ld2410_interface::msg::LD2410TargetDataFrame& radar_sensor);

    bool isPresenceDetectionChanged(const interaction_controller::msg::PresenceDetection& new_radar_presence_msg);

    void updateAndPublishPresenceDetection(const interaction_controller::msg::PresenceDetection& new_radar_presence_msg);

    rclcpp::Subscription<ld2410_interface::msg::LD2410TargetDataFrameArray>::SharedPtr radar_lld_sub_;
    rclcpp::Publisher<interaction_controller::msg::PresenceDetection>::SharedPtr radar_presence_pub_;
    interaction_controller::msg::PresenceDetection current_radar_presence_msg_;
};

}  // namespace radar_hld



#endif // RADAR_PRESENCE_HLD_INCLUDE_RADAR_PRESENCE_HLD_HPP_