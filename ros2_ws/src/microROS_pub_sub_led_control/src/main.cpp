#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/set_bool.h>
#include <std_msgs/msg/bool.h>

//#include <example_interfaces/srv/add_two_ints.h>
//#include <my_custom_message/msg/my_custom_message.h>

#include <my_custom_led_interface/srv/my_custom_led_control.h>

// Define pins for red and yellow LEDs
#define RED_LED_PIN 3
#define YELLOW_LED_PIN 4

// RCCHECK and RCSOFTCHECK Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} 

// Node, Publisher, and Service Handles
rcl_node_t node;
rcl_publisher_t led_status_publisher;
rcl_service_t led_control_service;
rcl_timer_t led_status_timer;
rclc_executor_t executor;

// Request and Response message allocations
std_srvs__srv__SetBool_Request led_request_msg;
std_srvs__srv__SetBool_Response led_response_msg;


// Status variables for the LEDs
bool red_led_status = false;
bool yellow_led_status = false;

// Error handle loop, prefents for now that the esp won't reboot
void error_loop()
{
    while (1)
    {
        delay(100);
    }
}

// Callback function for LED control service
void led_control_callback(const void *req, void *res)
{
    const std_srvs__srv__SetBool_Request *request = (const std_srvs__srv__SetBool_Request *)req;
    std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)res;

    if (request->data)
    {
        digitalWrite(RED_LED_PIN, HIGH);
        red_led_status = true;
    } 
    else
    {
        digitalWrite(RED_LED_PIN, LOW);
        red_led_status = false;
    }

    response->success = true;

    strcpy(response->message.data, (request->data) ? "Red LED turned on" : "Red LED turned off");
    response->message.size = strlen(response->message.data);

}

// Callback for the LED status publisher timer
void led_status_publish_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer != NULL)
    {
        std_msgs__msg__Bool msg;
        msg.data = red_led_status;
        RCSOFTCHECK(rcl_publish(&led_status_publisher, &msg, NULL));
    }
}

void setup()
{
    // Initialize LED pins
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);

    //init msg size
    led_response_msg.message.capacity = 20;
    led_response_msg.message.data = (char*) malloc(led_response_msg.message.capacity * sizeof(char));
    led_response_msg.message.size = 0;
    //strcpy(led_response_msg.message.data, "Hello World");
    //led_response_msg.message.size = strlen(led_response_msg.message.data);

    // Initialize serial or WiFi transport
#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
    Serial.begin(SERIAL_BAUDRATE);
    set_microros_serial_transports(Serial);
    delay(2000);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
    const char* ssid = WIFI_SSID;
    const char* password = WIFI_PASSWORD;
    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP);
    size_t agent_port = AGENT_PORT;

    digitalWrite(RED_LED_PIN, HIGH);
    red_led_status = true;
    set_microros_wifi_transports((char*) ssid, (char*) password, agent_ip, agent_port);
    digitalWrite(YELLOW_LED_PIN, HIGH);
    yellow_led_status = true;
#endif

    // Initialize micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "led_control_node", "", &support));

    // Initialize publisher
    // RCCHECK(rclc_publisher_init_default(
    //     &led_status_publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    //     "led_status"));

    // Initialize service
    RCCHECK(rclc_service_init_default(
        &led_control_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
        "control_led"));

    // Initialize timer
    // const unsigned int timer_period_ms = 1000;
    // RCCHECK(rclc_timer_init_default2(
    //     &led_status_timer,
    //     &support,
    //     RCL_MS_TO_NS(timer_period_ms),
    //     led_status_publish_callback,
    //     true));

    // Initialize executor
    //RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_service(
        &executor, 
        &led_control_service, 
        &led_request_msg, 
        &led_response_msg, 
        led_control_callback));
    //RCCHECK(rclc_executor_add_timer(&executor, &led_status_timer));
}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(100);
}