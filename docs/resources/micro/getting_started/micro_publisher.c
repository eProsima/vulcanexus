// C standard includes.
#include <stdio.h>
#include <unistd.h>

// micro-ROS general includes with general functionality.
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include subscriber message type.
#include <std_msgs/msg/string.h>

// Macros to handle micro-ROS return codes, error handling should be customized for the target system.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int) temp_rc); return 1; }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int) temp_rc); }}

// Set maximum publisher string length.
#define STRING_LEN 200

// The publisher and string message objects are declared as global so they are available on `timer_callback`.
rcl_publisher_t publisher;
std_msgs__msg__String msg;
int counter = 0;

// The ``timer_callback`` function will be in charge of publishing and increment the message data on each timer period.
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    if (timer != NULL)
    {
        // Update string message with new value and size before its published.
        sprintf(msg.data.data, "Hello from micro-ROS #%d", counter++);
        msg.data.size = strlen(msg.data.data);

        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        printf("Publishing: \"%s\"\n", msg.data.data);
    }
}

int main()
{
    // Get configured allocator.
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Initialize support object.
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Node name and namespace.
    const char * node_name = "minimal_micro_publisher";
    const char * node_namespace = "";

    // Create node with default configuration.
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, node_name, node_namespace, &support));

    // Topic name and type support.
    const char * topic_name = "topic";
    const rosidl_message_type_support_t * type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

    // Create publisher.
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        type_support,
        topic_name));

    // Initialize publisher message memory.
    char string_memory[STRING_LEN];
    msg.data.data = &string_memory[0];
    msg.data.size = 0;
    msg.data.capacity = STRING_LEN;

    // Create timer with its callback and its trigger period.
    rcl_timer_t timer;
    const unsigned int timer_period = 500;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_period),
        timer_callback));

    // Initialize executor with one handle.
    rclc_executor_t executor;
    const size_t number_of_handles = 1;
    RCCHECK(rclc_executor_init(&executor, &support.context, number_of_handles, &allocator));

    // Add timer callback to the executor.
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Spin forever
    RCSOFTCHECK(rclc_executor_spin(&executor));

    // Free used resources on spin error and exit.
    // This methods will free used memory even if connection with the Agent is lost.
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    return 1;
}