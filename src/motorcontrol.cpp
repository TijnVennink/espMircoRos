#include "common.h"
#include "logpublisher.h"

// Initialize motor control parameters
float motorSpeedInHz = 300.0f; // Default motor speed in Hz
float motorAcceleration = 300.0f; // Default motor acceleration in Hz^2

void initMotorControl(FastAccelStepper* stepper) {
    if (stepper == nullptr) {
        Serial.println("Motor control initialization failed: Stepper is null.");
        publish_log("Motor control initialization failed: Stepper is null.");
        return;
    }

    // Set initial speed and acceleration
    stepper->setSpeedInHz(motorSpeedInHz);
    stepper->setAcceleration(motorAcceleration);
    stepper->enableOutputs();

    Serial.println("Motor control initialized with default parameters.");
    publish_log("Motor control initialized with default parameters.");
}

void moveMotor(const std_msgs__msg__Float32* msg) {
    if (stepper == nullptr) {
        Serial.println("Move motor command failed: Stepper is null.");
        publish_log("Move motor command failed: Stepper is null.");
        return;
    }

    // Set speed and acceleration
    stepper->setSpeedInHz(motorSpeedInHz);
    stepper->setAcceleration(motorAcceleration);
    
    float targetSteps = msg->data;
    char targetStepsStr[20];
    dtostrf(targetSteps, 1, 2, targetStepsStr);
    publish_log(targetStepsStr);

    // Move the motor to the desired position
    stepper->move(targetSteps);

    // Wait for the move to complete or limit switch to trigger
    while (stepper->isRunning()) {
        if (digitalRead(limitSwitchPinX) == LOW) {
            Serial.println("Limit switch triggered. Stopping motor.");
            publish_log("Limit switch triggered. Stopping motor.");
            stepper->stopMove();
            break;
        }
        delay(10);
    }

    Serial.println("Motor movement completed.");
    publish_log("Motor movement completed.");
}
