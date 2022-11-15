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

// Set maximum received string length.
// Received publications with strings larger than this size will be discarded.
#define STRING_LEN 200

// Callback to handle incoming subscriber messages.
void subscription_callback(const void * msgin)
{
    // Message type shall be casted to expected type from void pointer.
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    printf("I heard: \"%s\"\n", msg->data.data);
}

int main()
{
    // Get configured allocator.
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Initialize support object.
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Node name and namespace.
    const char * node_name = "minimal_micro_subscriber";
    const char * node_namespace = "";

    // Create node.
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, node_name, node_namespace, &support));

    // Topic name and message type support.
    const char * topic_name = "topic";
    const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

    // Create subscriber.
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        type_support,
        topic_name));

    // Initialize executor.
    rclc_executor_t executor;
    const size_t number_of_handles = 1;
    RCCHECK(rclc_executor_init(&executor, &support.context, number_of_handles, &allocator));

    // Add subscriber callback.
    std_msgs__msg__String msg;
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    // Initialize subscriber message memory.
    char string_memory[STRING_LEN];
    msg.data.data = &string_memory[0];
    msg.data.size = 0;
    msg.data.capacity = STRING_LEN;

    // Spin forever
    RCSOFTCHECK(rclc_executor_spin(&executor));

    // Free used resources on spin error and exit.
    // This methods will free used memory even if connection with the Agent is lost.
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    return 1;
}