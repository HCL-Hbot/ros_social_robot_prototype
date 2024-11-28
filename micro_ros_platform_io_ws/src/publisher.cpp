#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/u_int8.h>

// RCCHECK and RCSOFTCHECK Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} 

//Node handles
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

//Publisher handles
rcl_publisher_t publisher_1;
rcl_timer_t timer_1;
std_msgs__msg__UInt8 msg_1;


//Handle for Timer, Service and Subscription (i.e. callback functions. A executor is needed for callback functions)
rclc_executor_t executor;





// Error handle loop, prevents that the esp won't reboot when rclc does not get initialized properly (e.g. no connection with agent)
void error_loop()
{
    while (1)
    {
        delay(100);
    }
}


// Callback function for counter timer
void counter_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {

    }
}


void setup()
{

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

//     // Initialize serial or WiFi transport
// #ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
//     Serial.begin(SERIAL_BAUDRATE);
//     set_microros_serial_transports(Serial);
//     delay(2000);
// #elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
//     const char* ssid = WIFI_SSID;
//     const char* password = WIFI_PASSWORD;
//     IPAddress agent_ip;
//     agent_ip.fromString(AGENT_IP);
//     size_t agent_port = AGENT_PORT;

//     digitalWrite(RED_LED_PIN, HIGH);
//     red_led_status = true;
//     set_microros_wifi_transports((char*) ssid, (char*) password, agent_ip, agent_port);
//     digitalWrite(YELLOW_LED_PIN, HIGH);
//     yellow_led_status = true;
// #endif

    // Initialize micro-ROS
    allocator = rcl_get_default_allocator();
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "controller_node_1", "", &support));


    // Initialize publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher_1,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "controller_1"));


    // // Initialize counter timer (1-second interval)
    // RCCHECK(rclc_timer_init_default2(
    //     &timer_1,
    //     &support,
    //     RCL_MS_TO_NS(1000),
    //     counter_timer_callback,
    //     true));

    // // Initialize executor with timer and service and subcriber 
    // // Third argument is total number_of_handles, which means the total number of subscriptions, timers, services, clients and guard conditions. Do not include the number of nodes and publishers!!!
    // RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); //number 3 is used because we have 1 timer, 1 service and 1 subscriber. 

    // //Order of using rlcl_executor_add_* function matters, this decides order of execution for service, pub, sub, timer.
    // //In this example first Service, then timer for publishing and then subscription.
 
    // RCCHECK(rclc_executor_add_timer(&executor, &timer_1));

}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); //timeout of 100 ms (which means this function blocks for 100 ms)
    
    //Alternatively this could be used instead of above:
    //rclc_executor_spin(&executor); //This function runs forever without coming back
    //Since we run this code in arduino framework, using the 'rclc_executor_spin_some' function in the loop function seems more appropriate.
}
