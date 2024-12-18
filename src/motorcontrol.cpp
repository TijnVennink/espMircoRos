#include "common.h"
#include "logpublisher.h"
#include <cmath>
#include <iostream>

// Define motor control parameters
float motorSpeedInHz = convertSpeedToHz(max_speed_mm_per_s);
float maxAccelerationInHz2 = convertAccelerationToHz2(max_speed_mm_per_s / 2);

// Function to convert distance in cm to steps
int distanceToSteps(float distance_cm)
{
    float circumference_mm = pulley_diameter * 3.14159;
    float distance_mm = distance_cm * 10.0f;
    float steps_per_mm = (pulses_per_rev * micro_step) / circumference_mm;
    return static_cast<int>(distance_mm * steps_per_mm);
}

// Function to convert speed from mm/s to Hz
float convertSpeedToHz(float speed_mm_per_s)
{
    float circumference_mm = pulley_diameter * 3.14159;
    float steps_per_mm = (pulses_per_rev * micro_step) / circumference_mm;
    return speed_mm_per_s * steps_per_mm;
}

// Function to convert acceleration from mm/s^2 to Hz^2
float convertAccelerationToHz2(float acceleration_mm_per_s2)
{
    float circumference_mm = pulley_diameter * 3.14159;
    float steps_per_mm = (pulses_per_rev * micro_step) / circumference_mm;
    return acceleration_mm_per_s2 * steps_per_mm;
}

// Function to calculate maximum degrees per second for all three joint axes
void calculateMaxDegreesPerSec(float max_speed_mm_per_s)
{
    // Calculate dX
    float X0 = sqrt(pow(a_x, 2) + pow(c_x - b_x, 2));
    float dX_length = max_speed_mm_per_s;
    float x_angle_rad = asin((X0 - dX_length) / c_x);
    float max_deg_per_sec_x = degrees(x_angle_rad);

    // Calculate dY
    float Y0 = sqrt(pow(a_y, 2) + pow(b_y - c_y, 2));
    float dY_length = max_speed_mm_per_s;
    float y_angle_rad = asin((Y0 - dY_length) / c_y);
    float max_deg_per_sec_y = degrees(y_angle_rad);

    // Calculate dZ
    float L_z = sqrt(pow(a_z, 2) + pow(b_z, 2));
    float beta_z = atan(a_z / b_z);
    float Z0 = 2 * L_z * sin(beta_z);
    float dZ_length = max_speed_mm_per_s;
    float z_angle_rad = asin((Z0 - dZ_length) / (2 * L_z * sin(beta_z)));
    float max_deg_per_sec_z = degrees(z_angle_rad);

    Serial.println("Max degrees/s for X axis: " + String(max_deg_per_sec_x));
    Serial.println("Max degrees/s for Y axis: " + String(max_deg_per_sec_y));
    Serial.println("Max degrees/s for Z axis: " + String(max_deg_per_sec_z));
    publish_log(("Max degrees/s for X axis: " + String(max_deg_per_sec_x)).c_str());
    publish_log(("Max degrees/s for Y axis: " + String(max_deg_per_sec_y)).c_str());
    publish_log(("Max degrees/s for Z axis: " + String(max_deg_per_sec_z)).c_str());
}

void initMotorControl(FastAccelStepper *stepperX, FastAccelStepper *stepperY, FastAccelStepper *stepperZ)
{
    if (stepperX == nullptr)
    {
        Serial.println("Motor control initialization failed: Stepper is null.");
        publish_log("Motor control initialization failed: Stepper is null.");
        return;
    }

    if (stepperY == nullptr)
    {
        Serial.println("Motor control initialization failed: Stepper is null.");
        publish_log("Motor control initialization failed: Stepper is null.");
        return;
    }

    if (stepperZ == nullptr)
    {
        Serial.println("Motor control initialization failed: Stepper is null.");
        publish_log("Motor control initialization failed: Stepper is null.");
        return;
    }

    // Set initial speed and acceleration
    float motorSpeedInHz = convertSpeedToHz(max_speed_mm_per_s);
    float maxAccelerationInHz2 = convertAccelerationToHz2(max_speed_mm_per_s * 2);
    stepperX->setSpeedInHz(motorSpeedInHz);
    stepperX->setAcceleration(maxAccelerationInHz2);
    stepperY->setSpeedInHz(motorSpeedInHz);
    stepperY->setAcceleration(maxAccelerationInHz2);
    stepperZ->setSpeedInHz(motorSpeedInHz);
    stepperZ->setAcceleration(maxAccelerationInHz2);

    // Log max degrees/s for all axes
    calculateMaxDegreesPerSec(max_speed_mm_per_s);

    Serial.println("Motor control initialized with default parameters.");
    publish_log(("Motor control initialized with parameters: speed = " + String(motorSpeedInHz) + " Hz, acceleration = " + String(maxAccelerationInHz2) + " Hz^2").c_str());
    publish_log(("With mm/s parameters: speed = " + String(max_speed_mm_per_s) + " mm/s, acceleration = " + String(max_speed_mm_per_s / 2) + " mm/s^2").c_str());
}

void stopAllMotors()
{
    if (stepperX != nullptr)
    {
        stepperX->stopMove();
    }
    if (stepperY != nullptr)
    {
        stepperY->stopMove();
    }
    if (stepperZ != nullptr)
    {
        stepperZ->stopMove();
    }
}

void moveMotorsXYZ(const std_msgs__msg__Float32 *msgX, const std_msgs__msg__Float32 *msgY, const std_msgs__msg__Float32 *msgZ)
{
    float targetStepsX = distanceToSteps(msgX->data);
    float targetStepsY = distanceToSteps(msgY->data);
    float targetStepsZ = distanceToSteps(msgZ->data);

    // char targetStepsStrX[20];
    // char targetStepsStrY[20];
    // char targetStepsStrZ[20];
    // dtostrf(targetStepsX, 1, 2, targetStepsStrX);
    // dtostrf(targetStepsY, 1, 2, targetStepsStrY);
    // dtostrf(targetStepsZ, 1, 2, targetStepsStrZ);
    // publish_log(targetStepsStrX);
    // publish_log(targetStepsStrY);
    // publish_log(targetStepsStrZ);

    // Convert to integer for motor movement
    int targetStepsIntX = static_cast<int>(targetStepsX);
    int targetStepsIntY = static_cast<int>(targetStepsY);
    int targetStepsIntZ = static_cast<int>(targetStepsZ);

    // Move the motors to the desired positions
    stepperX->moveTo(targetStepsIntX);
    stepperY->moveTo(targetStepsIntY);
    stepperZ->moveTo(targetStepsIntZ);

    // Wait for the moves to complete or limit switches to trigger
    while (stepperX->stepsToStop() > 10 || stepperY->stepsToStop() > 10 || stepperZ->stepsToStop() > 10)
    {
        if (digitalRead(limitSwitchPin) == HIGH)
        {
            Serial.println("Limit switch triggered. Stopping all motors.");
            publish_log("Limit switch triggered. Stopping all motors.");
            stopAllMotors();
            break;
        }
        delay(1);
    }

    // Serial.println("Motors movement completed.");
    // publish_log("Motors movement completed.");
}
