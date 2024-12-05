#include "common.h"
#include "logpublisher.h" 

// Initialize homing parameters
float homingSpeedInHz = 2.5f; // homing speed in Hz
float homingAcceleration = 20.0f; // homing acceleration in Hz^2

// Adjust homing functions
void initHoming(FastAccelStepper* stepper) {
    pinMode(limitSwitchPinX, INPUT_PULLUP);  // Set limit switch pin as input with pull-up resistor

    // Initialize stepper motor settings, etc.
    stepper->setSpeedInHz(homingSpeedInHz);
    stepper->setAcceleration(homingAcceleration);
    stepper->enableOutputs();
}

void homeStepper(FastAccelStepper* stepper) {
    Serial.println("Homing started...");
    publish_log("Homing started...");
    
    // Move stepper until limit switch is triggered (switch goes from LOW to HIGH)
    while (digitalRead(limitSwitchPinX) == LOW) {
        stepper->move(-1);  // Move stepper motor towards the limit switch (negative direction)
    }

    // Initialize stepper motor settings, etc.
    stepper->setSpeedInHz(homingSpeedInHz);
    stepper->setAcceleration(homingAcceleration);
    
    // Once triggered, stop the motor and perform any necessary actions
    stepper->setAcceleration(homingAcceleration*10.0);
    stepper->stopMove();
    Serial.println("Homing X complete. Limit switch triggered.");
    publish_log("Homing X complete. Limit switch triggered.");
    
    // Give the motor some time to fully stop
    delay(1000);  // Adjust delay if needed to give time for stop to take effect
    
    // Now move back a small distance if needed
    stepper->move(10);  // Move a little bit back after homing
    Serial.println("Moved back right?");
    
    while (stepper->isRunning()) {
        delay(100);  // Wait for the motor to finish
    }

    Serial.println("Homing process complete.");
    publish_log("Spotted: The system has been homed. Rumor has it, it's finally in its perfect position. What’s next? Stay tuned—XOXO, Gossip Bot.");
}
