// #include <Arduino.h>
// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_srvs/srv/set_bool.h>
// #include <std_msgs/msg/bool.h>

// // Define pins for red and yellow LEDs
// #define RED_LED_PIN 3
// #define YELLOW_LED_PIN 4

// // Node, Publisher, and Service Handles
// rcl_node_t node;
// rcl_publisher_t led_status_publisher;
// rcl_service_t led_control_service;
// rcl_timer_t led_status_timer;
// rclc_executor_t executor; // Global declaration for executor

// // Request and Response message allocations
// std_srvs__srv__SetBool_Request led_request_msg;
// std_srvs__srv__SetBool_Response led_response_msg;

// // Status variables for the LEDs
// bool red_led_status = false;
// bool yellow_led_status = false;

// // Callback function for LED control service
// void led_control_callback(const void *req, void *res)
// {
//     const std_srvs__srv__SetBool_Request *request = (const std_srvs__srv__SetBool_Request *)req;
//     std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)res;

//     if (request->data) {
//         digitalWrite(RED_LED_PIN, HIGH);    // Turn on Red LED
//         red_led_status = true;
//     } else {
//         digitalWrite(RED_LED_PIN, LOW);     // Turn off Red LED
//         red_led_status = false;
//     }

//     response->success = true;
//     response->message.data = (char *)malloc(25 * sizeof(char)); // Allocate memory for response message
//     strcpy(response->message.data, (request->data) ? "Red LED turned on" : "Red LED turned off");
// }

// // Callback for the LED status publisher timer
// void led_status_publish_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//     if (timer != NULL) {
//         std_msgs__msg__Bool msg;
//         msg.data = red_led_status;
//         rcl_publish(&led_status_publisher, &msg, NULL);
//     }
// }

// void setup()
// {
//     // Start de serial communciat
//     Serial.begin(115200);
//     set_microros_serial_transports(Serial);  // User serial port as transport
    
//     // Initialize LED pins
//     pinMode(RED_LED_PIN, OUTPUT);
//     pinMode(YELLOW_LED_PIN, OUTPUT);
//     digitalWrite(RED_LED_PIN, LOW);
//     digitalWrite(YELLOW_LED_PIN, LOW);

//     // Initialize micro-ROS
//     rcl_allocator_t allocator = rcl_get_default_allocator();
//     rclc_support_t support;

//     // Initialize the node and other micro-ROS components
//     rclc_support_init(&support, 0, NULL, &allocator);
//     rclc_node_init_default(&node, "led_control_node", "", &support);

//     // Initialize publisher
//     rclc_publisher_init_default(
//         &led_status_publisher,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
//         "led_status");

//     // Initialize service
//     rclc_service_init_default(
//         &led_control_service,
//         &node,
//         ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
//         "control_led");

//     // Initialize timer with new API
//     const unsigned int timer_period_ms = 1000; // 1 second
//     rclc_timer_init_default2(
//         &led_status_timer,
//         &support,
//         RCL_MS_TO_NS(timer_period_ms),
//         led_status_publish_callback,
//         true); // Autostart timer

//     // Initialize executor to manage callbacks
//     rclc_executor_init(&executor, &support.context, 2, &allocator);

//     // Use allocated request and response message for the service
//     rclc_executor_add_service(
//         &executor, 
//         &led_control_service, 
//         &led_request_msg, 
//         &led_response_msg, 
//         led_control_callback);

//     rclc_executor_add_timer(&executor, &led_status_timer);
// }

// void loop()
// {
//     // micro-ROS runs the executor for callbacks and timers
//     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
//     delay(10);
// }
