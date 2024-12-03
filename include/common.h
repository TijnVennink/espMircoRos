#ifndef COMMON_H
#define COMMON_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <FastAccelStepper.h>
#include <std_msgs/msg/float32.h>


// Shared variables
extern FastAccelStepperEngine engine;
extern FastAccelStepper* stepper;

// Homing parameters
extern float homingSpeedInHz;
extern float homingAcceleration;

// Pin definitions
#define stepPinStepperX 5
#define dirPinStepperX 6
#define enablePinStepperX 7
#define limitSwitchPinX 8

// Function prototypes
void initHoming(FastAccelStepper* stepper);
void homeStepper(FastAccelStepper* stepper);

#endif // COMMON_H
