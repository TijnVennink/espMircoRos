#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <std_msgs/msg/float32.h>
#include "logpublisher.h" 
#include <std_msgs/msg/float32_multi_array.h>
#include <stdlib.h>
#include <queue>
#include "common.h"
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw/qos_profiles.h>

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

// Define the MotorCommand struct
struct MotorCommand {
    int targetStepsX;
    int targetStepsY;
    int targetStepsZ;
};

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
FastAccelStepper* stepperZ = nullptr;

bool homing_complete = false;

void error_loop() {
    while (1) {
        delay(100);
    }
}

// Function to move motors and set the `motorsBusy` flag
void moveMotorsXYZ(int targetStepsX, int targetStepsY, int targetStepsZ); // Forward declaration

// Callback function for motor commands
void motor_callback(const void* msgin) {
    const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)msgin;

    if (msg->data.size < 3) {
        if (!homing_complete) {
            publish_log("All motor inputs are None. Starting homing sequence.");
            homeSteppers(stepperX, stepperY, stepperZ);
            homing_complete = true;
        } else {
            publish_log("Invalid message size. Expected 3 values.");
            publish_log("Homing already completed. No motor commands to execute.");
        }
        return;
    }

    // Convert distances to steps
    int targetStepsX = static_cast<int>(msg->data.data[0] * steps_per_cm);
    int targetStepsY = static_cast<int>(msg->data.data[1] * steps_per_cm);
    int targetStepsZ = static_cast<int>(msg->data.data[2] * steps_per_cm);

    // Add the command to the queue
    commandQueue.push({targetStepsX, targetStepsY, targetStepsZ});
}

// Function to process the next command in the queue
void processNextCommand() {
    if (!motorsBusy && !commandQueue.empty()) {
        // Get the next command from the queue
        std::array<int, 3> cmdArray = commandQueue.front();
        MotorCommand cmd = {cmdArray[0], cmdArray[1], cmdArray[2]};
        commandQueue.pop();

        // Move motors to the new target
        moveMotorsXYZ(cmd.targetStepsX, cmd.targetStepsY, cmd.targetStepsZ);
    }
}

// Function to check if all motors are done moving
void checkMotorsStatus() {
    if (motorsBusy &&
        !stepperX->isRunning() &&
        !stepperY->isRunning() &&
        !stepperZ->isRunning()) {
        // All motors are done moving
        motorsBusy = false;
        // Process the next command
        processNextCommand();
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

        // Initialize stepper Z
    stepperZ = engine.stepperConnectToPin(stepPinStepperZ);
    if (!stepperZ) {
        Serial.println("Failed to initialize stepper Z!");
        publish_log("Failed to initialize stepper Z!");
        return;
    }

    stepperZ->setDirectionPin(dirPinStepperZ);
    stepperZ->setEnablePin(enablePinStepperZ);
    stepperZ->setAutoEnable(false);
    stepperZ->enableOutputs();

    // ROS setup
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "main_node", "", &support));

    // Initialize the log publisher (defined in logpublisher.cpp)
    init_log_publisher(&node);
    publish_log("Log publisher booted");

    // Initialize motor system
    initMotorControl(stepperX, stepperY, stepperZ);
    publish_log("Motor control initialized");

    // Initialize homing system
    initHoming(stepperX, stepperY, stepperZ);
    publish_log("Homing initialized");

    // Define a custom QoS profile with a larger queue size
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 100; // Set the queue size to 50

    // Initialize the subscriber with the custom QoS profile
    RCCHECK(rclc_subscription_init(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_command",
        &custom_qos
    ));

    // Initialize the executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &motor_msg, &motor_callback, ALWAYS));


    Serial.println("ROS setup completed. Waiting for commands...");
    publish_log("Almighty not robotic ARM legendary ROS setup completed. Waiting for commands...");
}

void loop() {
    delay(10);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    checkMotorsStatus();
}
