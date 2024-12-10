#include "common.h"
#include "logpublisher.h"

// Initialize motor control parameters
float motorSpeedInHz = 30.0f; // Default motor speed in Hz
float motorAcceleration = 50.0f; // Default motor acceleration in Hz^2

void initMotorControl(FastAccelStepper* stepperX, FastAccelStepper* stepperY, FastAccelStepper* stepperZ) {
    if (stepperX == nullptr) {
        Serial.println("Motor control initialization failed: Stepper is null.");
        publish_log("Motor control initialization failed: Stepper is null.");
        return;
    }

    if (stepperY == nullptr) {
        Serial.println("Motor control initialization failed: Stepper is null.");
        publish_log("Motor control initialization failed: Stepper is null.");
        return;
    }

    if (stepperZ == nullptr) {
        Serial.println("Motor control initialization failed: Stepper is null.");
        publish_log("Motor control initialization failed: Stepper is null.");
        return;
    }

    // Set initial speed and acceleration
    stepperX->setSpeedInHz(motorSpeedInHz);
    stepperX->setAcceleration(motorAcceleration);
    stepperY->setSpeedInHz(motorSpeedInHz);
    stepperY->setAcceleration(motorAcceleration);
    stepperZ->setSpeedInHz(motorSpeedInHz);
    stepperZ->setAcceleration(motorAcceleration);

    Serial.println("Motor control initialized with default parameters.");
    publish_log("Motor control initialized with default parameters.");
}

void moveMotorX(const std_msgs__msg__Float32* msg) {
    if (stepperX == nullptr) {
        Serial.println("Move motor command failed: Stepper is null.");
        publish_log("Move motor command failed: Stepper is null.");
        return;
    }

    // Set speed and acceleration
    stepperX->setSpeedInHz(motorSpeedInHz);
    stepperX->setAcceleration(motorAcceleration);
    
    float targetSteps = msg->data;
    char targetStepsStr[20];
    dtostrf(targetSteps, 1, 2, targetStepsStr);
    publish_log(targetStepsStr);

    // Convert to integer for motor movement
    int targetStepsInt = static_cast<int>(targetSteps);
    // Move the motor to the desired position
    stepperX->move(targetSteps);
    

    // Wait for the move to complete or limit switch to trigger
    while (stepperX->isRunning()) {
        if (digitalRead(limitSwitchPin) == HIGH) {
            Serial.println("Limit switch triggered. Stopping motor.");
            publish_log("Limit switch(X) triggered. Stopping motor.");
            stepperX->stopMove();
            break;
        }
        delay(10);
    }

    Serial.println("Motor movement completed.");
    publish_log("Motor movement completed.");
}


void moveMotorY(const std_msgs__msg__Float32* msg) {
    // Set speed and acceleration
    stepperY->setSpeedInHz(motorSpeedInHz);
    stepperY->setAcceleration(motorAcceleration);
    
    float targetSteps = msg->data;
    char targetStepsStr[20];
    dtostrf(targetSteps, 1, 2, targetStepsStr);
    publish_log(targetStepsStr);

    // Convert to integer for motor movement
    int targetStepsInt = static_cast<int>(targetSteps);
    // Move the motor to the desired position
    stepperY->move(targetSteps);
    

    // Wait for the move to complete or limit switch to trigger
    while (stepperY->isRunning()) {
        if (digitalRead(limitSwitchPin) == HIGH) {
            Serial.println("Limit switch triggered. Stopping motor.");
            publish_log("Limit switch(Y) triggered. Stopping motor.");
            stepperY->stopMove();
            break;
        }
        delay(10);
    }

    Serial.println("Motor movement completed.");
    publish_log("Motor movement completed.");
}


void moveMotorZ(const std_msgs__msg__Float32* msg) {
    // Set speed and acceleration
    stepperZ->setSpeedInHz(motorSpeedInHz);
    stepperZ->setAcceleration(motorAcceleration);
    
    float targetSteps = msg->data;
    char targetStepsStr[20];
    dtostrf(targetSteps, 1, 2, targetStepsStr);
    publish_log(targetStepsStr);

    // Convert to integer for motor movement
    int targetStepsInt = static_cast<int>(targetSteps);
    // Move the motor to the desired position
    stepperZ->move(targetSteps);
    

    // Wait for the move to complete or limit switch to trigger
    while (stepperZ->isRunning()) {
        if (digitalRead(limitSwitchPin) == HIGH) {
            Serial.println("Limit switch triggered. Stopping motor.");
            publish_log("Limit switch(Z) triggered. Stopping motor.");
            stepperZ->stopMove();
            break;
        }
        delay(10);
    }

    Serial.println("Motor movement completed.");
    publish_log("Motor movement completed.");
}


