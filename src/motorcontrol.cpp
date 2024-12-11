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

void stopAllMotors() {
    if (stepperX != nullptr) {
        stepperX->stopMove();
    }
    if (stepperY != nullptr) {
        stepperY->stopMove();
    }
    if (stepperZ != nullptr) {
        stepperZ->stopMove();
    }
}

void moveMotorX(const std_msgs__msg__Float32* msg) {
    if (stepperX == nullptr) {
        Serial.println("Move motor command failed: Stepper is null.");
        publish_log("Move motor command failed: Stepper is null.");
        return;
    }

    stopAllMotors(); // Stop any ongoing movement

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
    stepperX->move(targetStepsInt);
    
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
    if (stepperY == nullptr) {
        Serial.println("Move motor command failed: Stepper is null.");
        publish_log("Move motor command failed: Stepper is null.");
        return;
    }

    stopAllMotors(); // Stop any ongoing movement

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
    stepperY->move(targetStepsInt);
    
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
    if (stepperZ == nullptr) {
        Serial.println("Move motor command failed: Stepper is null.");
        publish_log("Move motor command failed: Stepper is null.");
        return;
    }

    stopAllMotors(); // Stop any ongoing movement

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
    stepperZ->move(targetStepsInt);
    
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

void moveMotorsXYZ(const std_msgs__msg__Float32* msgX, const std_msgs__msg__Float32* msgY, const std_msgs__msg__Float32* msgZ) {
    if (stepperX == nullptr || stepperY == nullptr || stepperZ == nullptr) {
        Serial.println("Move motors command failed: One or more steppers are null.");
        publish_log("Move motors command failed: One or more steppers are null.");
        return;
    }

    stopAllMotors(); // Stop any ongoing movement

    // Set speed and acceleration for all motors
    stepperX->setSpeedInHz(motorSpeedInHz);
    stepperX->setAcceleration(motorAcceleration);
    stepperY->setSpeedInHz(motorSpeedInHz);
    stepperY->setAcceleration(motorAcceleration);
    stepperZ->setSpeedInHz(motorSpeedInHz);
    stepperZ->setAcceleration(motorAcceleration);

    float targetStepsX = msgX->data;
    float targetStepsY = msgY->data;
    float targetStepsZ = msgZ->data;

    char targetStepsStrX[20];
    char targetStepsStrY[20];
    char targetStepsStrZ[20];
    dtostrf(targetStepsX, 1, 2, targetStepsStrX);
    dtostrf(targetStepsY, 1, 2, targetStepsStrY);
    dtostrf(targetStepsZ, 1, 2, targetStepsStrZ);
    publish_log(targetStepsStrX);
    publish_log(targetStepsStrY);
    publish_log(targetStepsStrZ);

    // Convert to integer for motor movement
    int targetStepsIntX = static_cast<int>(targetStepsX);
    int targetStepsIntY = static_cast<int>(targetStepsY);
    int targetStepsIntZ = static_cast<int>(targetStepsZ);

    // Move the motors to the desired positions
    stepperX->move(targetStepsIntX);
    stepperY->move(targetStepsIntY);
    stepperZ->move(targetStepsIntZ);

    // Wait for the moves to complete or limit switches to trigger
    while (stepperX->isRunning() || stepperY->isRunning() || stepperZ->isRunning()) {
        if (digitalRead(limitSwitchPin) == HIGH) {
            Serial.println("Limit switch triggered. Stopping all motors.");
            publish_log("Limit switch triggered. Stopping all motors.");
            stopAllMotors();
            break;
        }
        delay(10);
    }

    Serial.println("Motors movement completed.");
    publish_log("Motors movement completed.");
}


