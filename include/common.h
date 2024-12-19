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
extern FastAccelStepper* stepperZ;

#define RMW_UXRCE_MAX_NODES 2
#define SUBSCRIBER_NUMBER 1

// Homing parameters
extern float homingSpeedInHz;
extern float homingAcceleration;

// Max speed in mm/s
const float max_speed_mm_per_s = 20.0f;

// Motor control parameters
extern float motorSpeedInHz;
extern float maxAccelerationInHz2;

// Pin definitions
#define stepPinStepperX 5
#define dirPinStepperX 6
#define enablePinStepperX 7
#define limitSwitchPin 8 //Same pin for now

#define stepPinStepperY 9
#define dirPinStepperY 10
#define enablePinStepperY 11

#define stepPinStepperZ 15
#define dirPinStepperZ 16
#define enablePinStepperZ 17

// Define constants
const float pulley_diameter = 3.0; // Diameter of the pulley in cm
const int pulses_per_rev = 200; // Number of pulses per revolution of the motor
const int micro_step = 1; // Microstepping setting of the motor driver

float circumference_cm = 3.14159 * pulley_diameter;

const float a_x = 10.800;
const float b_x = 8.374;
const float c_x = 12.437;

const float a_y = 10.800;
const float b_y = 11.937;
const float c_y = 8.374;

const float a_z = 10.800;
const float b_z = 8.165;

// Function prototypes
void init_float32_multi_array(std_msgs__msg__Float32MultiArray *msg, float *buffer, size_t size);
void cleanup_float32_multi_array(std_msgs__msg__Float32MultiArray *msg);
void initHoming(FastAccelStepper* stepperX, FastAccelStepper* stepperY, FastAccelStepper* stepperZ);
void homeStepperAxis(FastAccelStepper* stepperX);
void homeStepperAxis(FastAccelStepper* stepper, const char* stepperName);
void homeSteppers(FastAccelStepper* stepperX, FastAccelStepper* stepperY, FastAccelStepper* stepperZ);
void initMotorControl(FastAccelStepper* stepperX, FastAccelStepper* stepperY, FastAccelStepper* stepperZ);
void moveMotorX(const std_msgs__msg__Float32* msg);
void moveMotorY(const std_msgs__msg__Float32* msg);
void moveMotorZ(const std_msgs__msg__Float32* msg);
void moveMotorsXYZ(const std_msgs__msg__Float32* msgX, const std_msgs__msg__Float32* msgY, const std_msgs__msg__Float32* msgZ);

// Function prototypes for speed and acceleration conversion
float convertSpeedToHz(float speed_mm_per_s);
float convertAccelerationToHz2(float acceleration_mm_per_s2);
int distanceToSteps(float distance_cm);

#endif // COMMON_H
