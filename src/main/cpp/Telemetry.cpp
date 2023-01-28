#include "Telemetry.h"

Telemetry::Telemetry(DriveTrain *drvt)
{
    drivebase = drvt;                                   // Reference our created drivetrain
    gyro = new ctre::phoenix::sensors::PigeonIMU(16);   // Create our gyro object with can ID 16
    gyro->SetYaw(0.0);                                  // Set the x angle to 0
    drivetrain_data = new SparkMaxPacket;               // Allocate memory for the packet struct
    timer_interval = 100;                               // Set the timer to capture a snapshot every 100ms(10x second)
    snapshot_timer = new Timer((double)timer_interval); // Set the timer to take a snapshot every 100ms(10x second)
    max_rewind_time = 5;                                // Set the max rewind time to be 5s
    max_rewind_steps = 5 * 1000 / timer_interval;       // Set the max rewind steps to be proportional to the interval time and overall time
}

SparkMaxPacket *Telemetry::exportTelemetry()
{
    return drivetrain_data; // Return our struct pointer to expose writing to telemetry data
}

void Telemetry::runMetrics()
{
    if (snapshot_timer->getTimer())
    {                                 // Capture a snapshot of the robots data every 100ms
        drivebase->updateTelemetry(); // Load in motor and encoder data from the drivetrain class
        captureSnapshot();            // Capture the updated drivetrain data
        manageRewind();               // Take data from snapshot and save it to our rewind buffer
    }
    // Do calculations and such here
}

void Telemetry::captureSnapshot()
{
    new_capture = new Snapshot;
    new_capture->left_pos = drivetrain_data->encoder_position_1 * drivetrain_data->encoder_position_2 / 2;  // Average data values
    new_capture->right_pos = drivetrain_data->encoder_position_3 * drivetrain_data->encoder_position_4 / 2; // Average data values
    new_capture->left_speed = drivetrain_data->motor_power_1 * drivetrain_data->motor_power_2 / 2;          // Average left speed value
    new_capture->right_speed = drivetrain_data->motor_power_3 * drivetrain_data->motor_power_4 / 2;         // Average right speed value
    new_capture->x_rot = gyro->GetYaw();                                                                    // Get x rotation
    instruction_queue.push(new_capture);                                                                    // Add snapshot struct to new_capture
}

void Telemetry::manageRewind()
{
    rewind_steps.push_back(new_capture);
    if (rewind_steps.size() > max_rewind_steps)
    {
        rewind_steps.pop_front(); // Remove the snapshot from the front of the list. Shouldn't have to worry about memory management,
                                  // since the popped data will still be pointed to by the instruction queue.
    }
}

// TODO: Add methodd to get the current instruction stack whether by copying or freezing new data addition so we can pass data to drivetrain input

Telemetry::~Telemetry()
{
    delete gyro;            // Free the gyro memory
    delete drivetrain_data; // Free the telemetry struct
    delete snapshot_timer;  // Free timer
}