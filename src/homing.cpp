#include "common.h"
#include "logpublisher.h" 

// Initialize homing parameters
float homingSpeedInHz = 10.0f; // homing speed in Hz
float homingAcceleration = 50.0f; // homing acceleration in Hz^2


// Adjust homing functions
void initHoming(FastAccelStepper* stepperX, FastAccelStepper* stepperY, FastAccelStepper* stepperZ)  {
    pinMode(limitSwitchPin, INPUT_PULLUP);  // Set limit switch pin as input with pull-up resistor
    // Initialize stepper motor settings, etc.
    stepperX->setSpeedInHz(homingSpeedInHz);
    stepperX->setAcceleration(homingAcceleration);
    stepperY->setSpeedInHz(homingSpeedInHz);
    stepperY->setAcceleration(homingAcceleration);
    stepperZ->setSpeedInHz(homingSpeedInHz);
    stepperZ->setAcceleration(homingAcceleration);
}

void homeSteppers(FastAccelStepper* stepperX, FastAccelStepper* stepperY, FastAccelStepper* stepperZ) {
    Serial.println("Homing started...");
    publish_log("Homing started...");
    
    // Initialize stepper motor settings, etc.
    stepperX->setSpeedInHz(homingSpeedInHz);
    stepperX->setAcceleration(homingAcceleration);
    stepperY->setSpeedInHz(homingSpeedInHz);
    stepperY->setAcceleration(homingAcceleration);
    stepperZ->setSpeedInHz(homingSpeedInHz);
    stepperZ->setAcceleration(homingAcceleration);

    // Home x-axis
    publish_log("-------Called homing of X-axis!-------");
    homeStepperAxis(stepperX, "Stepper X");
    publish_log("-------Finished homing of X-axis!-------");

    // Home y-axis
    publish_log("-------Called homing of Y-axis!-------");
    homeStepperAxis(stepperY, "Stepper Y");
    publish_log("-------Finished homing of Y-axis!-------");

    // Home z-axis
    publish_log("-------Called homing of Z-axis!-------");
    homeStepperAxis(stepperZ, "Stepper Z");
    publish_log("-------Finished homing of Z-axis!-------");

    publish_log("Spotted: The system has been homed. Rumor has it, it's finally in its perfect position. What’s next? Stay tuned—XOXO, Gossip Bot.");
}

void homeStepperAxis(FastAccelStepper* stepper, const char* stepperName) {
    // Negative direction first
    char logMessage[100];
    snprintf(logMessage, sizeof(logMessage), "Homing %s: Moving in negative direction", stepperName);
    publish_log(logMessage);

    // Move stepper until limit switch is triggered (switch goes from LOW to HIGH)
    while (digitalRead(limitSwitchPin) == LOW) {
        stepper->move(-1);  // Move stepper motor towards the limit switch (negative direction)
    }

    // Once triggered, stop the motor and perform any necessary actions
    stepper->setAcceleration(homingAcceleration * 10.0);
    stepper->stopMove();

    snprintf(logMessage, sizeof(logMessage), "Homing %s: Limit switch triggered", stepperName);
    publish_log(logMessage);

    // Now move back a small distance if needed
    stepper->move(10);  // Move a little bit back after homing

    // Give the motor some time to fully stop
    delay(1000);  // Adjust delay if needed to give time for stop to take effect

    while (stepper->isRunning()) {
        delay(100);  // Wait for the motor to finish
    }

    snprintf(logMessage, sizeof(logMessage), "Homing %s: At negative limit position", stepperName);
    publish_log(logMessage);

    // Some delay for the message to be published
    delay(1000);  // Adjust delay if needed to give time for stop to take effect

    // Positive direction
    snprintf(logMessage, sizeof(logMessage), "Homing %s: Moving in positive direction", stepperName);
    publish_log(logMessage);

    // Move stepper until limit switch is triggered (switch goes from LOW to HIGH)
    while (digitalRead(limitSwitchPin) == LOW) {
        stepper->move(1);  // Move stepper motor towards the limit switch (positive direction)
    }

    // Once triggered, stop the motor and perform any necessary actions
    stepper->setAcceleration(homingAcceleration * 10.0);
    stepper->stopMove();

    snprintf(logMessage, sizeof(logMessage), "Homing %s: Limit switch triggered", stepperName);
    publish_log(logMessage);

    // Now move back a small distance if needed
    stepper->move(10);  // Move a little bit back after homing

    // Give the motor some time to fully stop
    delay(1000);  // Adjust delay if needed to give time for stop to take effect

    while (stepper->isRunning()) {
        delay(100);  // Wait for the motor to finish
    }

    snprintf(logMessage, sizeof(logMessage), "Homing %s: At positive limit position", stepperName);
    publish_log(logMessage);

    publish_log("Homed axis");
}