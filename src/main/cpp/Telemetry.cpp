#include "Telemetry.h"

Telemetry::Telemetry(DriveTrain *drvt)
{
    drivebase = drvt;                                 // Reference our created drivetrain
    gyro = new ctre::phoenix::sensors::PigeonIMU(16); // Create our gyro object with can ID 16
    gyro->SetYaw(0.0);                                // Set the x angle to 0
    drivetrain_data = new SparkMaxPacket;             // Allocate memory for the packet struct
    snapshot_timer = new Timer(100);                  // Set the timer to take a snapshot every 100ms(10x second)
}

SparkMaxPacket *Telemetry::exportTelemetry()
{
    return drivetrain_data; // Return our struct pointer to expose writing to telemetry data
}

void Telemetry::runMetrics()
{
    if(snapshot_timer->getTimer()){//Capture a snapshot of the robots data every 100ms
        captureSnapshot();
    }
    // Do calculations and such here
}

void Telemetry::captureSnapshot()
{
    drivebase->updateTelemetry();
    struct Snapshot *temp = new Snapshot;
    temp->left_pos = drivetrain_data->encoder_position_1 * drivetrain_data->encoder_position_2 / 2;  // Average data values
    temp->right_pos = drivetrain_data->encoder_position_3 * drivetrain_data->encoder_position_4 / 2; // Average data values
    temp->left_speed = drivetrain_data->motor_power_1 * drivetrain_data->motor_power_2 / 2;          // Average left speed value
    temp->right_speed = drivetrain_data->motor_power_3 * drivetrain_data->motor_power_4 / 2;         // Average right speed value
    temp->x_rot = gyro->GetYaw();                                                                    // Get x rotation
    instruction_stack.push(temp);                                                                    // Add snapshot struct to temp
}

Telemetry::~Telemetry()
{
    delete gyro;            // Free the gyro memory
    delete drivetrain_data; // Free the telemetry struct
    delete snapshot_timer;  // Free timer
}