#ifndef LOGPUBLISHER_H
#define LOGPUBLISHER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

// Declare the publisher globally
extern rcl_publisher_t log_publisher;

// Function to initialize the publisher
void init_log_publisher(rcl_node_t* node);

// Function to publish a log message
void publish_log(const char* message);

#endif // LOGPUBLISHER_H
