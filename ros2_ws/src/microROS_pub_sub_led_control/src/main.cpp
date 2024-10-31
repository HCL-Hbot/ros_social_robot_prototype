#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <my_custom_led_interface/srv/my_custom_led_control.h>
#include <std_msgs/msg/int32.h>

// Define pins for red and yellow LEDs
#define RED_LED_PIN 3
#define YELLOW_LED_PIN 4

// RCCHECK and RCSOFTCHECK Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} 

// Node, Publisher, and Service Handles
rcl_node_t node;
rcl_publisher_t led_status_publisher;
rcl_publisher_t counter_publisher; // Publisher voor de counter
rcl_service_t led_control_service;
rcl_timer_t led_status_timer;
rcl_timer_t counter_timer;
rclc_executor_t executor;

rcl_allocator_t allocator;
rclc_support_t support;

// Request and Response message allocations
my_custom_led_interface__srv__MyCustomLedControl_Request led_request_msg;
my_custom_led_interface__srv__MyCustomLedControl_Response led_response_msg;
std_msgs__msg__Int32 counter_msg;

// Status variables for the LEDs
volatile bool red_led_status = false;
volatile bool yellow_led_status = false;
//int counter_value = 0; // Counter waarde die elke seconde wordt opgehoogd

// Error handle loop, prevents for now that the esp won't reboot
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
    const my_custom_led_interface__srv__MyCustomLedControl_Request *request = (const my_custom_led_interface__srv__MyCustomLedControl_Request *)req;
    my_custom_led_interface__srv__MyCustomLedControl_Response *response = (my_custom_led_interface__srv__MyCustomLedControl_Response *)res;

    response->success = true;

    if (request->led_color == 1)
    {
        digitalWrite(RED_LED_PIN, request->led_on);
        red_led_status = request->led_on;
        strcpy(response->message.data, (red_led_status) ? "Red LED turned on" : "Red LED turned off");
    }
    else if (request->led_color == 2)
    {
        digitalWrite(YELLOW_LED_PIN, request->led_on);
        yellow_led_status = request->led_on;
        strcpy(response->message.data, (yellow_led_status) ? "Yellow LED turned on" : "Yellow LED turned off");
    }
    else
    {
        response->success = false;
        strcpy(response->message.data, "Invalid request");
    }  

    response->message.size = strlen(response->message.data);
}

// Callback function for counter timer
void counter_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&counter_publisher, &counter_msg, NULL));
        ++counter_msg.data;
    }
}

void setup()
{
    // Initialize LED pins
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);

    //init msg sizes
    led_response_msg.message.capacity = 25;
    led_response_msg.message.data = (char*) malloc(led_response_msg.message.capacity * sizeof(char));
    led_response_msg.message.size = 0;
    
    counter_msg.data = 0;

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
    allocator = rcl_get_default_allocator();
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "led_control_node", "", &support));

    // Initialize LED control service
    RCCHECK(rclc_service_init_default(
        &led_control_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(my_custom_led_interface, srv, MyCustomLedControl),
        "control_led"));

    // Initialize counter publisher
    RCCHECK(rclc_publisher_init_default(
        &counter_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "counter"));

    // Initialize counter timer (1-second interval)
    RCCHECK(rclc_timer_init_default2(
        &counter_timer,
        &support,
        RCL_MS_TO_NS(1000),
        counter_timer_callback,
        true));

    // Initialize executor with added timer and service
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_service(
        &executor, 
        &led_control_service, 
        &led_request_msg, 
        &led_response_msg, 
        led_control_callback));
    RCCHECK(rclc_executor_add_timer(&executor, &counter_timer));
}

void loop()
{
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
