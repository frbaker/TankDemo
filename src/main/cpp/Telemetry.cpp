#include "Telemetry.h"

Telemetry::Telemetry(DriveTrain *drvt)
{
    drivebase = drvt;                                 // Reference our created drivetrain
    gyro = new ctre::phoenix::sensors::PigeonIMU(16); // Create our gyro object with can ID 16
    gyro->SetYaw(0.0);                                // Set the x angle to 0
    drivetrain_data = new SparkMaxPacket;             // Allocate memory for the packet struct
}

SparkMaxPacket *Telemetry::exportTelemetry()
{
    return drivetrain_data; // Return our struct pointer to expose writing to telemetry data
}

void Telemetry::runMetrics()
{
    // Update our speed and position data
    drivebase->updateTelemetry();
    // Do calculations and such here
}

Telemetry::~Telemetry()
{
    delete gyro;            // Free the gyro memory
    delete drivetrain_data; // Free the telemetry struct
}