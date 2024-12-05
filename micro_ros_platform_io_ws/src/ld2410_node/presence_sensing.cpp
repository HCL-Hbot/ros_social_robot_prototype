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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  delay(2000); //wait for serial monitor to open

  Serial.println("Starting radar node");

  #ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL //Start node in serial communciation mode.
    const UartConfig ros_serial_config {0,0,0,SERIAL_BAUDRATE}; //UART=0, Baudrate=256000 
    radar_node.reset(new RadarLd2410Manager<NUM_SENSORS>("ld2410_manager_node", "ld2410_target_frames", ros_serial_config, 0));
  #elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) //Start node in wifi coumminciation mode.
    WifiConfig wifi_config {AGENT_PORT, AGENT_IP, WIFI_SSID, WIFI_PASSWORD};
    radar_node.reset(new RadarLd2410Manager<NUM_SENSORS>("ld2410_manager_node", "ld2410_target_frames", wifi_config, 0));
    Serial.println("Wifi config set");
  #endif


  Serial.println("Radar node created");
  /*Order of config decides the sensor id. So sensor with uart 1 in this case will get id 0*/
  const std::array<UartConfig, NUM_SENSORS> radarConfigs = {{
    {L_RADAR_RX, L_RADAR_TX, 1, 256000}, // Radar Sensor 1:  RX=38, TX=37, UART=1, Baudrate=256000 
    {R_RADAR_RX, R_RADAR_TX, 2, 256000}  
  }};
  
  Serial.println("Initializing radars");
  radar_node->initializeRadars(radarConfigs);

  Serial.println("Radar node initialized");

  while(!radar_node->isAgentAvialable()) //ping the agent to check if it is available.
  {
    Serial.println("Agent not avialable. Retrying...");
  }
  
  bool initResult = radar_node->initMicroRos();
  
  if(!initResult)
  {
    Serial.println("Failed to init micro-ROS");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); //turn on the LED to indicate failure
    while (true){}; //loop forever
  }
  Serial.println("micro-ROS initialized");
}

void loop()
{
  if(radar_node->mostRecentpublishFailed() && !radar_node->isAgentAvialable())
  {
    Serial.println("Agent not avialable...");
    bool result = radar_node->clean();
    //cleaning seems to be failing, but in reality it is not.
    if(!result)
    {
      Serial.println("Failed to clean micro-ROS");
    }
    else
    {
      Serial.println("Cleaned micro-ROS");
    }
    while(!radar_node->isAgentAvialable()) //ping the agent to check if it is available.
    {
      Serial.println("Agent not avialable. Retrying!...");
    }
    result = radar_node->initMicroRos();
    if(!result)
    {
      Serial.println("Failed to init micro-ROS");
    }
    else
    {
      Serial.println("micro-ROS initialized");
    }
  }
  else
  {
    radar_node->spinSome(RCL_MS_TO_NS(1000)); //spin for 1000ms
  }

}