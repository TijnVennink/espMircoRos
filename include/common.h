#ifndef COMMON_H
#define COMMON_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <FastAccelStepper.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

// Shared variables
extern FastAccelStepperEngine engine;
extern FastAccelStepper* stepperX;
extern FastAccelStepper* stepperY;

#define RMW_UXRCE_MAX_NODES 2
#define SUBSCRIBER_NUMBER 1

// Homing parameters
extern float homingSpeedInHz;
extern float homingAcceleration;

// Pin definitions
#define stepPinStepperX 5
#define dirPinStepperX 6
#define enablePinStepperX 7
#define limitSwitchPin 8 //Same pin for now

#define stepPinStepperY 9
#define dirPinStepperY 10
#define enablePinStepperY 11

#define stepPinStepperZ 13
#define dirPinStepperZ 14
#define enablePinStepperZ 15
 
// Function prototypes
void init_float32_multi_array(std_msgs__msg__Float32MultiArray *msg, float *buffer, size_t size);
void cleanup_float32_multi_array(std_msgs__msg__Float32MultiArray *msg);
void initHoming(FastAccelStepper* stepperX, FastAccelStepper* stepperY);
void homeStepperAxis(FastAccelStepper* stepperX);
void homeSteppers(FastAccelStepper* stepperX, FastAccelStepper* stepperY);
void initMotorControl(FastAccelStepper* stepperX, FastAccelStepper* stepperY);
void moveMotorX(const std_msgs__msg__Float32* msg);
void moveMotorY(const std_msgs__msg__Float32* msg);
void moveMotorZ(const std_msgs__msg__Float32* msg);

#endif // COMMON_H
