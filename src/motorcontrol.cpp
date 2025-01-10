#include "common.h"
#include "logpublisher.h"
#include <cmath>
#include <iostream>

// Define motor control parameters
float motorSpeedInHz = convertSpeedToHz(max_speed_mm_per_s);
float maxAccelerationInHz2 = convertAccelerationToHz2(max_speed_mm_per_s / 20);

// Global variables for precomputed constants
float steps_per_cm;

// Function to convert distance in cm to steps
int distanceToSteps(float distance_cm)
{
    float circumference_cm = pulley_diameter * 3.14159; // Calculate circumference in cm
    float steps_per_cm = (pulses_per_rev * micro_step) / circumference_cm; // Calculate steps per cm
    return static_cast<int>(distance_cm * steps_per_cm); // Convert distance to steps
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
    if (stepperX == nullptr || stepperY == nullptr || stepperZ == nullptr)
    {
        Serial.println("Motor control initialization failed: One or more steppers are null.");
        publish_log("Motor control initialization failed: One or more steppers are null.");
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

    // Precompute steps per cm
    float circumference_cm = pulley_diameter * 3.14159;
    steps_per_cm = (pulses_per_rev * micro_step) / circumference_cm;

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
    // Convert distances to steps using precomputed steps_per_cm
    int targetStepsIntX = static_cast<int>(msgX->data * steps_per_cm);
    int targetStepsIntY = static_cast<int>(msgY->data * steps_per_cm);
    int targetStepsIntZ = static_cast<int>(msgZ->data * steps_per_cm);

    // Move the motors to the desired positions
    stepperX->moveTo(targetStepsIntX);
    stepperY->moveTo(targetStepsIntY);
    stepperZ->moveTo(targetStepsIntZ);
}
