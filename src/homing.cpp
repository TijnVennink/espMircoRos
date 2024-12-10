#include "common.h"
#include "logpublisher.h" 
#include "homingpublisher.h" 

// Initialize homing parameters
float homingSpeedInHz = 10.0f; // homing speed in Hz
float homingAcceleration = 50.0f; // homing acceleration in Hz^2


// Adjust homing functions
void initHoming(FastAccelStepper* stepperX, FastAccelStepper* stepperY) {
    pinMode(limitSwitchPin, INPUT_PULLUP);  // Set limit switch pin as input with pull-up resistor
    // Initialize stepper motor settings, etc.
    stepperX->setSpeedInHz(homingSpeedInHz);
    stepperX->setAcceleration(homingAcceleration);
    stepperY->setSpeedInHz(homingSpeedInHz);
    stepperY->setAcceleration(homingAcceleration);
}

void homeSteppers(FastAccelStepper* stepperX, FastAccelStepper* stepperY) {
    Serial.println("Homing started...");
    publish_log("Homing started...");
    
    // Initialize stepper motor settings, etc.
    stepperX->setSpeedInHz(homingSpeedInHz);
    stepperX->setAcceleration(homingAcceleration);
    stepperY->setSpeedInHz(homingSpeedInHz);
    stepperY->setAcceleration(homingAcceleration);

    // Home x-axis
    publish_log("-------Called homing of X-axis!-------");
    homeStepperAxis(stepperX);
    publish_log("-------Finished homing of X-axis!-------");

    // Home y-axis
    publish_log("-------Called homing of Y-axis!-------");
    homeStepperAxis(stepperY);
    publish_log("-------Finished homing of Y-axis!-------");


}


void homeStepperAxis(FastAccelStepper* stepper) {
    // Negative direction first
    publish_log("Moving in negative direction on axis");
    // Move stepper until limit switch is triggered (switch goes from LOW to HIGH)
    while (digitalRead(limitSwitchPin) == LOW) {
        stepper->move(-1);  // Move stepper motor towards the limit switch (negative direction)
    }

    // Once triggered, stop the motor and perform any necessary actions
    stepper->setAcceleration(homingAcceleration*10.0);
    stepper->stopMove();

    Serial.println("Limit switch triggered.");
    publish_log("Limit switch triggered.");

    // // Publish that this axis is at the negative limit position
    // publish_homing_log("At negative limit position");
    
    // Now move back a small distance if needed
    stepper->move(10);  // Move a little bit back after homing

    // Give the motor some time to fully stop
    delay(1000);  // Adjust delay if needed to give time for stop to take effect
    
    while (stepper->isRunning()) {
        delay(100);  // Wait for the motor to finish
    }

    Serial.println("Limit switch triggered.");
    publish_log("At negative limit position");
    
    // // Publish that this axis is at the positive limit position
    // publish_homing_log("At positive limit position");


    // Some delay for the message to be published
    delay(1000);  // Adjust delay if needed to give time for stop to take effect


    // Negative direction first
    publish_log("Moving in positive direction on axis");
    // Move stepper until limit switch is triggered (switch goes from LOW to HIGH)
    while (digitalRead(limitSwitchPin) == LOW) {
        stepper->move(1);  // Move stepper motor towards the limit switch (negative direction)
    }

    // Once triggered, stop the motor and perform any necessary actions
    stepper->setAcceleration(homingAcceleration*10.0);
    stepper->stopMove();

    Serial.println("Limit switch triggered.");
    publish_log("Limit switch triggered.");
    
    // Now move back a small distance if needed
    stepper->move(10);  // Move a little bit back after homing

    // Give the motor some time to fully stop
    delay(1000);  // Adjust delay if needed to give time for stop to take effect
    
    while (stepper->isRunning()) {
        delay(100);  // Wait for the motor to finish
    }

    Serial.println("Limit switch triggered.");
    publish_log("At positive limit position");
    // PUBLISH THIS HERE!

    publish_log("Homed axis");

}