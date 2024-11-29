#include <Arduino.h>
#include "FastAccelStepper.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// Stepper configuration
#define stepPinStepper 4
#define dirPinStepper 5
#define enablePinStepper 6

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Micro-ROS setup
rcl_publisher_t publisher;
rcl_subscription_t subscriber;

std_msgs__msg__Float32 received_msg;
std_msgs__msg__Float32 publish_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

// Subscriber callback function
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  if (stepper) {
    stepper->stopMove(); // Stop any ongoing movement
    stepper->moveTo(static_cast<int>(msg->data)); // Move to the received position
    Serial.print("Moving stepper to: ");
    Serial.println(msg->data);
  }
}

void setup() {
  // Serial for debugging
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(1000);

  // Stepper motor initialization
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setSpeedInHz(1000);
    stepper->setAcceleration(1000);
    delay(4000);
    stepper->enableOutputs();
  }

  // Micro-ROS initialization
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_control_node", "", &support));

  // Subscriber setup
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_command"));

  // Executor setup
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // Single subscription
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &received_msg,
    &subscription_callback,
    ALWAYS));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
