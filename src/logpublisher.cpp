#include "logpublisher.h"
#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

rcl_publisher_t log_publisher; // Define the publisher

void init_log_publisher(rcl_node_t* node) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    
    // Initialize the publisher
    rcl_ret_t ret = rclc_publisher_init_default(
        &log_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "log_topic"
    );
    
    if (ret != RCL_RET_OK) {
        Serial.println("Failed to initialize log publisher");
    } else {
        Serial.println("Log publisher initialized successfully");
    }
}


void publish_log(const char* message) {
    std_msgs__msg__String msg;
    msg.data.data = (char*) message;
    msg.data.size = strlen(message);
    msg.data.capacity = msg.data.size + 1;

    rcl_ret_t ret = rcl_publish(&log_publisher, &msg, NULL);
    if (ret == RCL_RET_OK) {
        Serial.print("Published: ");
        Serial.println(message);
    } else {
        Serial.println("Failed to publish message.");
    }
}
