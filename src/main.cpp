#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <std_msgs/msg/float32.h>
#include "logpublisher.h" 
#include <std_msgs/msg/float32_multi_array.h>
#include <stdlib.h>
#include "common.h"

// ROS setup
rcl_subscription_t subscriber;
// std_msgs__msg__Float32 motor_msg;

/////////////////////////////////////////////////////
// Try with multi array
void init_float32_multi_array(std_msgs__msg__Float32MultiArray *msg, float *buffer, size_t size) {
    // Initialize the message
    std_msgs__msg__Float32MultiArray__init(msg);

    // Initialize the data field
    msg->data.data = buffer;
    msg->data.capacity = size;
    msg->data.size = 0;

    // Initialize the layout field
    msg->layout.dim.capacity = 1; // Single-dimensional array
    msg->layout.dim.size = 1;
    msg->layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(
        msg->layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

    std_msgs__msg__MultiArrayDimension *dim = &msg->layout.dim.data[0];
    dim->size = size;
    dim->stride = size;
    dim->label.capacity = 20;
    dim->label.size = 0;
    dim->label.data = (char *)malloc(dim->label.capacity * sizeof(char));

    snprintf(dim->label.data, dim->label.capacity, "motor_commands");
    dim->label.size = strlen(dim->label.data);
}

void cleanup_float32_multi_array(std_msgs__msg__Float32MultiArray *msg) {
    // Free label memory
    for (size_t i = 0; i < msg->layout.dim.size; ++i) {
        free(msg->layout.dim.data[i].label.data);
    }
    // Free dimension array memory
    free(msg->layout.dim.data);

    // Finalize the message
    std_msgs__msg__Float32MultiArray__fini(msg);
}

// Define the message and buffer for pre-allocation
float buffer[3];
std_msgs__msg__Float32MultiArray motor_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

FastAccelStepperEngine engine;
FastAccelStepper* stepperX = nullptr;
FastAccelStepper* stepperY = nullptr;


bool homing_complete = false;

void error_loop() {
    while (1) {
        delay(100);
    }
}

// Motor Subscriber callback for controlling x, y, z motors
void motor_callback(const void* msgin) {
    const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)msgin;

    publish_log("motor_callback called");

    // Check the size of the array
    if (msg->data.size < 3) {
        publish_log("Invalid message size. Expected 3 values.");
        return;
    }

    // Temporary structures to pass single motor data
    std_msgs__msg__Float32 x_msg;
    std_msgs__msg__Float32 y_msg;
    std_msgs__msg__Float32 z_msg;

    // Extract motor commands and populate temporary messages
    x_msg.data = msg->data.data[0]; // X motor control
    y_msg.data = msg->data.data[1]; // Y motor control
    z_msg.data = msg->data.data[2]; // Z motor control

    // Check if all inputs are None (interpreted as 0.0f for this example)
    bool x_none = x_msg.data == 0.0f;
    bool y_none = y_msg.data == 0.0f;
    bool z_none = z_msg.data == 0.0f;

    if (x_none && y_none && z_none) {
        if (!homing_complete) {
            publish_log("All motor inputs are None. Starting homing sequence.");
            homeSteppers(stepperX, stepperY);
            homing_complete = true;  // Mark homing as complete
        } else {
            publish_log("Homing already completed. No motor commands to execute.");
        }
        return;
    }

    // Control each motor if a command is present
    if (!x_none) {
        publish_log("Moving X motor.");
        moveMotorX(&x_msg);
    }

    if (!y_none) {
        publish_log("Moving Y motor.");
        moveMotorY(&y_msg);
    }

    if (!z_none) {
        publish_log("Moving Z motor.");
        // No MotorZ attached yet :()
        // moveMotorZ(&z_msg);
    }
}

void setup() {
    init_float32_multi_array(&motor_msg, buffer, 3);
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(1000);

    // Initialize stepper X
    engine.init();
    stepperX = engine.stepperConnectToPin(stepPinStepperX);
    if (!stepperX) {
        Serial.println("Failed to initialize stepper X!");
        publish_log("Failed to initialize stepper X!");
        return;
    }

    stepperX->setDirectionPin(dirPinStepperX);
    stepperX->setEnablePin(enablePinStepperX);
    stepperX->setAutoEnable(false);
    stepperX->enableOutputs();

    // Initialize stepper Y
    stepperY = engine.stepperConnectToPin(stepPinStepperY);
    if (!stepperY) {
        Serial.println("Failed to initialize stepper Y!");
        publish_log("Failed to initialize stepper Y!");
        return;
    }

    stepperY->setDirectionPin(dirPinStepperY);
    stepperY->setEnablePin(enablePinStepperY);
    stepperY->setAutoEnable(false);
    stepperY->enableOutputs();

    // ROS setup
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "main_node", "", &support));

    // Initialize the log publisher (defined in logpublisher.cpp)
    init_log_publisher(&node);
    publish_log("Log publisher booted");

    // Initialize motor system
    initMotorControl(stepperX, stepperY);
    publish_log("Motor control initialized");

    // Initialize homing system
    initHoming(stepperX, stepperY);
    publish_log("Homing initialized");

    // Initialize the subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
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
