#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <my_custom_led_interface/srv/my_custom_led_control.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int8.h>

// Define pins for red and yellow LEDs
#define RED_LED_PIN 3
#define YELLOW_LED_PIN 4

// RCCHECK and RCSOFTCHECK Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} 

//Node handles
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

//Publisher handles
rcl_publisher_t led_status_publisher;

//Service handles
rcl_service_t led_control_service;

// Subscriber handles
rcl_subscription_t brightness_subscriber; // Toegevoegd voor de nieuwe subscriber

//Handle for Timer
rcl_timer_t led_status_timer;

//Handle for Timer, Service and Subscription (i.e. callback functions. A executor is needed for callback functions)
rclc_executor_t executor;

// Request and Response message allocations
my_custom_led_interface__srv__MyCustomLedControl_Request led_request_msg;
my_custom_led_interface__srv__MyCustomLedControl_Response led_response_msg;

// Publish message allocation
std_msgs__msg__String led_pub_msg;
const uint8_t PUB_MSG_CAPACITY = (71+4+4+3); //calculation for max message size

// LED brightness control message
std_msgs__msg__UInt8 brightness_msg;

// Status variables for the LEDs
volatile bool red_led_status = false;
volatile bool yellow_led_status = false;

// Error handle loop, prevents that the esp won't reboot when rclc does not get initialized properly (e.g. no connection with agent)
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
    const uint8_t brightness = (request->led_on) ? brightness_msg.data : 0;

    if (request->led_color == 1)
    {
        analogWrite(RED_LED_PIN, brightness);

        red_led_status = request->led_on;
        strcpy(response->message.data, (red_led_status) ? "Red LED turned on" : "Red LED turned off");
    }
    else if (request->led_color == 2)
    {
        analogWrite(YELLOW_LED_PIN, brightness);

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

// Callback function for LED brightness subscriber
void brightness_callback(const void *msg_in)
{
    const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msg_in;
    const uint8_t brightness = msg->data;
    
    if(red_led_status)
    {
        analogWrite(RED_LED_PIN, brightness);
    }

    if(yellow_led_status)
    {
        analogWrite(YELLOW_LED_PIN, brightness); 
    }
}

// Callback function for counter timer
void counter_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        char buffer[PUB_MSG_CAPACITY];
        sprintf(buffer, "Red LED is %s and Yellow LED is %s. Brightness value for all LED is %d" , (red_led_status) ? "On" : "Off",  (yellow_led_status) ? "On" : "Off", brightness_msg.data);
        strcpy(led_pub_msg.data.data, buffer);
        led_pub_msg.data.size = strlen(led_pub_msg.data.data);
        RCSOFTCHECK(rcl_publish(&led_status_publisher, &led_pub_msg, NULL));
    }
}

void setup()
{
    // Initialize LED pins
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);

    //Init msg sizes
    led_response_msg.message.capacity = 25;
    led_response_msg.message.data = (char*) malloc(led_response_msg.message.capacity * sizeof(char));
    //Alternatively this could be used instead of above:
    //led_response_msg.message.data = (char*) allocator.allocate(led_response_msg.message.capacity * sizeof(char), allocator.state);
    //By default allocator uses malloc as implementation (see rcl_get_default_allocator()), custom allocator(s) could also be made for specific platforms. 
    //In this example we just use malloc directly, because allacotor is not initialized at this stage of the code.
    led_response_msg.message.size = 0;
    

    led_pub_msg.data.capacity = PUB_MSG_CAPACITY;
    led_pub_msg.data.data = (char*) malloc(led_pub_msg.data.capacity * sizeof(char));
    led_pub_msg.data.size = 0;

    brightness_msg.data = 255; // full brightness for all LEDS as initial value.

    
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

    // Initialize publisher
    RCCHECK(rclc_publisher_init_default(
        &led_status_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "led_status"));

   // Initialize brightness subscriber
    RCCHECK(rclc_subscription_init_default(
        &brightness_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "led_brightness"));

    // Initialize counter timer (1-second interval)
    RCCHECK(rclc_timer_init_default2(
        &led_status_timer,
        &support,
        RCL_MS_TO_NS(1000),
        counter_timer_callback,
        true));

    // Initialize executor with timer and service and subcriber 
    // Third argument is total number_of_handles, which means the total number of subscriptions, timers, services, clients and guard conditions. Do not include the number of nodes and publishers!!!
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator)); //number 3 is used because we have 1 timer, 1 service and 1 subscriber. 

    //Order of using rlcl_executor_add_* function matters, this decides order of execution for service, pub, sub, timer.
    //In this example first Service, then timer for publishing and then subscription.
    RCCHECK(rclc_executor_add_service(
        &executor, 
        &led_control_service, 
        &led_request_msg, 
        &led_response_msg, 
        led_control_callback));
    RCCHECK(rclc_executor_add_timer(&executor, &led_status_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &brightness_subscriber, &brightness_msg, &brightness_callback, ON_NEW_DATA));

}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); //timeout of 100 ms (which means this function blocks for 100 ms)
    
    //Alternatively this could be used instead of above:
    //rclc_executor_spin(&executor); //This function runs forever without coming back
    //Since we run this code in arduino framework, using the 'rclc_executor_spin_some' function in the loop function seems more appropriate.
}
