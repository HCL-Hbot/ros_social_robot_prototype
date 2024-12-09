#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <memory>

#include "radar_ld2410_manager.hpp"

//left (from the robot)
#define L_RADAR_RX 38
#define L_RADAR_TX 37

//right (from the robot)
#define R_RADAR_RX 40
#define R_RADAR_TX 39

#define NODE_NAME "ld2410_manager_node"
#define RADAR_PUBLISH_TOPIC_NAME "ld2410_target_frames"
#define DEVICE_ID 0
#define PIN_LED_INDICATOR 48
#define NUM_SENSORS 2

std::unique_ptr<RadarLd2410Manager<NUM_SENSORS>> radar_node;

void setup(void)
{

  /*Order of config decides the sensor id. So sensor with uart 1 in this case will get id 0*/
  const std::array<UartConfig, NUM_SENSORS> radarConfigs = {{
    {L_RADAR_RX, L_RADAR_TX, 1, 256000}, // Radar Sensor 1:  RX=38, TX=37, UART=1, Baudrate=256000 
    {R_RADAR_RX, R_RADAR_TX, 2, 256000}  
  }};

  #ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL //Start node in serial communciation mode.
    const UartConfig ros_serial_config {0,0,0,SERIAL_BAUDRATE}; //UART=0, Baudrate=256000 
    radar_node.reset(new RadarLd2410Manager<NUM_SENSORS>(NODE_NAME, RADAR_PUBLISH_TOPIC_NAME, radarConfigs, ros_serial_config, DEVICE_ID, PIN_LED_INDICATOR));
  #elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) //Start node in wifi coumminciation mode.
    const WifiConfig wifi_config {AGENT_PORT, AGENT_IP, WIFI_SSID, WIFI_PASSWORD};
    radar_node.reset(new RadarLd2410Manager<NUM_SENSORS>(NODE_NAME, RADAR_PUBLISH_TOPIC_NAME, radarConfigs, wifi_config, DEVICE_ID, PIN_LED_INDICATOR));
  #endif
}

void loop()
{
  radar_node->updateStateMachine();
}