#ifndef HOMINGPUBLISHER_H
#define HOMINGPUBLISHER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

// Declare the publisher globally
extern rcl_publisher_t homing_publisher;

// Function to initialize the publisher
void init_homing_publisher(rcl_node_t* node);
// Function to publish a log message
void publish_homing_log(const char* message);

#endif // HOMINGPUBLISHER_H
