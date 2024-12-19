#ifndef COMMON_H
#define COMMON_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <FastAccelStepper.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <queue>

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
extern const float pulley_diameter;
extern const int pulses_per_rev;
extern const int micro_step;

extern float circumference_cm;

extern const float a_x;
extern const float b_x;
extern const float c_x;

extern const float a_y;
extern const float b_y;
extern const float c_y;

extern const float a_z;
extern const float b_z;

// Flag to track if motors are currently moving
extern bool motorsBusy;

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
void checkMotorsStatus();

// Define the command queue
extern std::queue<std::array<int, 3>> commandQueue;

// Global variables for precomputed constants
extern float steps_per_cm;

// Function prototypes for speed and acceleration conversion
float convertSpeedToHz(float speed_mm_per_s);
float convertAccelerationToHz2(float acceleration_mm_per_s2);
int distanceToSteps(float distance_cm);

#endif // COMMON_H
