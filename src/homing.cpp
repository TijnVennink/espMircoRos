#include "common.h"
#include "logpublisher.h" 
#include <utility>

// Define the limit position variables
int x_nega_limit_pos = -900;
int x_posi_limit_pos = 900;
int y_nega_limit_pos = -900;
int y_posi_limit_pos = 900;
int z_nega_limit_pos = -900;
int z_posi_limit_pos = 900;

// Adjust homing functions
void initHoming(FastAccelStepper* stepperX, FastAccelStepper* stepperY, FastAccelStepper* stepperZ)  {
    pinMode(limitSwitchPin, INPUT_PULLUP);  // Set limit switch pin as input with pull-up resistor
    // Initialize stepper motor settings, etc.
    stepperX->setSpeedInHz(motorSpeedInHz);
    stepperX->setAcceleration(maxAccelerationInHz2);
    stepperY->setSpeedInHz(motorSpeedInHz);
    stepperY->setAcceleration(maxAccelerationInHz2);
    stepperZ->setSpeedInHz(motorSpeedInHz);
    stepperZ->setAcceleration(maxAccelerationInHz2);

    publish_log("Initialized homing positions (should be 0s)");
    char logMessage[100];
    snprintf(logMessage, sizeof(logMessage), "X: %d, Y: %d, Z: %d", stepperX->getCurrentPosition(), stepperY->getCurrentPosition(), stepperZ->getCurrentPosition());
    publish_log(logMessage);
}

void homeSteppers(FastAccelStepper* stepperX, FastAccelStepper* stepperY, FastAccelStepper* stepperZ) {
    Serial.println("Homing started...");
    publish_log("Homing started...");
    
    // Initialize stepper motor settings, etc.
    stepperX->setSpeedInHz(motorSpeedInHz/20);
    stepperX->setAcceleration(maxAccelerationInHz2);
    stepperY->setSpeedInHz(motorSpeedInHz/20);
    stepperY->setAcceleration(maxAccelerationInHz2);
    stepperZ->setSpeedInHz(motorSpeedInHz/20);
    stepperZ->setAcceleration(maxAccelerationInHz2);

    // Home x-axis
    publish_log("-------Called homing of X-axis!-------");
    auto x_limits = homeStepperAxis(stepperX, "Stepper X");
    x_nega_limit_pos = x_limits.first;
    x_posi_limit_pos = x_limits.second;
    publish_log("-------Finished homing of X-axis!-------");
    delay(3000);
    // Home y-axis
    publish_log("-------Called homing of Y-axis!-------");
    auto y_limits = homeStepperAxis(stepperY, "Stepper Y");
    y_nega_limit_pos = y_limits.first;
    y_posi_limit_pos = y_limits.second;
    publish_log("-------Finished homing of Y-axis!-------");
    delay(3000);
    // Home z-axis
    publish_log("-------Called homing of Z-axis!-------");
    auto z_limits = homeStepperAxis(stepperZ, "Stepper Z");
    z_nega_limit_pos = z_limits.first;
    z_posi_limit_pos = z_limits.second;
    publish_log("-------Finished homing of Z-axis!-------");
    delay(3000);
    publish_log("Spotted: The system has been homed. Rumor has it, it's finally in its perfect position. What’s next? Stay tuned—XOXO, Gossip Bot.");
}

std::pair<int, int> homeStepperAxis(FastAccelStepper* stepper, const char* stepperName) {
    // Negative direction first
    char logMessage[100];
    snprintf(logMessage, sizeof(logMessage), "Homing %s: Moving in negative direction", stepperName);
    publish_log(logMessage);

    // Move stepper until limit switch is triggered (switch goes from LOW to HIGH)
    while (digitalRead(limitSwitchPin) == HIGH) {
        stepper->move(-1);  // Move stepper motor towards the limit switch (negative direction)
    }

    // Once triggered, stop the motor and perform any necessary actions
    stepper->setAcceleration(maxAccelerationInHz2 * 10.0);
    stepper->stopMove();

    snprintf(logMessage, sizeof(logMessage), "Homing %s: Limit switch triggered", stepperName);
    publish_log(logMessage);

    // Now move back a small distance if needed
    stepper->move(5);  // Move a little bit back after homing

    // Give the motor some time to fully stop
    delay(1000);  // Adjust delay if needed to give time for stop to take effect

    while (stepper->isRunning()) {
        delay(100);  // Wait for the motor to finish
    }
    int nega_limit_pos = stepper->getCurrentPosition();
    snprintf(logMessage, sizeof(logMessage), "Homing %s: At negative limit position %d", stepperName, nega_limit_pos);
    publish_log(logMessage);

    // Some delay for the message to be published
    delay(1000);  // Adjust delay if needed to give time for stop to take effect
    
    stepper->moveTo(0.0); 
    delay(3000);  // Adjust delay if needed to give time for stop to take effect

    // Positive direction
    snprintf(logMessage, sizeof(logMessage), "Homing %s: Moving in positive direction", stepperName);
    publish_log(logMessage);

    // Move stepper until limit switch is triggered (switch goes from LOW to HIGH)
    while (digitalRead(limitSwitchPin) == HIGH) {
        stepper->move(1);  // Move stepper motor towards the limit switch (positive direction)
    }

    // Once triggered, stop the motor and perform any necessary actions
    stepper->setAcceleration(maxAccelerationInHz2 * 10.0);
    stepper->stopMove();

    snprintf(logMessage, sizeof(logMessage), "Homing %s: Limit switch triggered", stepperName);
    publish_log(logMessage);

    // Now move back a small distance if needed
    stepper->move(-5);  // Move a little bit back after homing

    // Give the motor some time to fully stop
    delay(1000);  // Adjust delay if needed to give time for stop to take effect

    while (stepper->isRunning()) {
        delay(100);  // Wait for the motor to finish
    }
    int posi_limit_pos = stepper->getCurrentPosition();
    snprintf(logMessage, sizeof(logMessage), "Homing %s: At positive limit position %d", stepperName, posi_limit_pos);
    publish_log(logMessage);

    stepper->moveTo(0.0); 
    delay(3000);  // Adjust delay if needed to give time for stop to take effect
    publish_log("Homed axis");

    return std::make_pair(nega_limit_pos, posi_limit_pos);
}
