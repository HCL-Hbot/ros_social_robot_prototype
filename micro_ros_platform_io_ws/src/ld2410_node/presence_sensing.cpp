#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <memory>

#include "radar_ld2410_manager.hpp"

//links (van de robot)
#define L_RADAR_RX 38
#define L_RADAR_TX 37

//rechts (van de robot)
#define R_RADAR_RX 40
#define R_RADAR_TX 39

#define NUM_SENSORS 2

std::unique_ptr<RadarLd2410Manager<NUM_SENSORS>> radar_node; //omdat we 2 HARDWARE UART beschikbaar hebben.. TODO met een flag configuren

void setup(void)
{
  #ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL //Start node in serial communciation mode.
    const UartConfig ros_serial_config {0,0,0,SERIAL_BAUDRATE}; //UART=0, Baudrate=256000 
    radar_node.reset(new RadarLd2410Manager<NUM_SENSORS>("ld2410_manager_node",ros_serial_config));
  #elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) //Start node in wifi coumminciation mode.
    WifiConfig wifi_config {AGENT_PORT, AGENT_IP, WIFI_SSID, WIFI_PASSWORD};
    radar_node.reset(new RadarLd2410Manager<NUM_SENSORS>("ld2410_manager_node", wifi_config));
  #endif

  /*Order of config decides the sensor id. So sensor with uart 1 in this case will get id 0*/
  const std::array<UartConfig, NUM_SENSORS> radarConfigs = {{
    {L_RADAR_RX, L_RADAR_TX, 1, 256000}, // Radar Sensor 1:  RX=38, TX=37, UART=1, Baudrate=256000 
    {R_RADAR_RX, R_RADAR_TX, 2, 256000}  
  }};
  
  radar_node->initializeRadars(radarConfigs);

}

void loop()
{
  radar_node->spin();
  delay(1000);
}