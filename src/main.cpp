#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <std_msgs/msg/float32.h>
#include "common.h"
#include "logpublisher.h" 

// ROS setup
rcl_subscription_t subscriber;
std_msgs__msg__Float32 motor_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;

bool homing_complete = false;

void error_loop() {
    while (1) {
        delay(100);
    }
}

// Motor Subscriber callback
void motor_callback(const void* msgin) {
    const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
    if (msg->data == 0.0f && !homing_complete) { // Check if homing is not complete
        publish_log("Homing command received.");
        homeStepper(stepper);
        homing_complete = true;  // Set flag to true after homing
    } else if (msg->data != 0.0f) {
        publish_log("moveMotor(msg) called");
        moveMotor(msg);
    } else {
        publish_log("Please JP and Iris home first");
    }
}

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(1000);

    // Initialize stepper
    engine.init();
    stepper = engine.stepperConnectToPin(stepPinStepperX);
    if (!stepper) {
        Serial.println("Failed to initialize stepper!");
        return;
    }

    stepper->setDirectionPin(dirPinStepperX);
    stepper->setEnablePin(enablePinStepperX);
    stepper->setAutoEnable(true);

    // ROS setup
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "main_node", "", &support));

    // Initialize the log publisher (defined in logpublisher.cpp)
    init_log_publisher(&node);
    publish_log("Log publisher booted");

    // Initialize motor system
    initMotorControl(stepper);
    publish_log("Motor control initialized");

    // Initialize homing system
    initHoming(stepper);
    publish_log("Homing initialized");

    // Initialize the subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "motor_command"
    ));

    // Initialize the executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &motor_msg, &motor_callback, ALWAYS));

    Serial.println("ROS setup completed. Waiting for commands...");
    publish_log("Almighty not robotic ARM legendary ROS setup completed. Waiting for commands...");
}

void loop() {
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
