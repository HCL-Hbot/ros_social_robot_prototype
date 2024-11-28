#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <array>

class MicroRosNode
{
    public:
    MicroRosNode();
    ~MicroRosNode();

    private:
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;

    std::array<rcl_publisher_t, RMW_UXRCE_MAX_PUBLISHERS> publisher;
    //std::array<rcl_publisher_t publisher_1;
    //Handle for Timer, Service and Subscription (i.e. callback functions. A executor is needed for callback functions)
    rclc_executor_t executor;
};