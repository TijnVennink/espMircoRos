#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <FastAccelStepper.h>

// Function prototypes
void moveMotorsXYZ(FastAccelStepper* stepperX, FastAccelStepper* stepperY, FastAccelStepper* stepperZ, float x, float y, float z);

#endif // MOTORCONTROL_H