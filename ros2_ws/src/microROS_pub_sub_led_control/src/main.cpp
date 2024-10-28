#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>


#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
#include <WiFi.h>  // (Supports only WPA2-Personal)


const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
IPAddress agent_ip = IPAddress().fromString(AGENT_IP);
const uint16_t agent_port = AGENT_PORT;

#endif

#define RED_LED_PIN 3
#define YELLOW_LED_PIN 4

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop()
{
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup()
{
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
  Serial.begin(SERIAL_BAUDRATE);
  set_microros_serial_transports(Serial);
  delay(2000);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
  IPAddress agent_ip(10, 42, 0, 1);

  size_t agent_port = 8888;

  digitalWrite(RED_LED_PIN, HIGH);

  set_microros_wifi_transports((char*) ssid, (char*) password, agent_ip, agent_port);
  
  digitalWrite(YELLOW_LED_PIN, HIGH);

#endif

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback,
    true));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop()
{
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}