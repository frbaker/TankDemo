#include "Telemetry.h"

/**
 * @brief Construct a new Telemetry:: Telemetry object
 *
 * @param drvt Takes a pointer to the drivetrain object to enable drivetrain data passthrough
 */
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

/**
 * @brief Expose the telemetry data packet to the drivetrain so that the drivetrain can populate it with data.
 *
 * @return SparkMaxPacket*
 */
SparkMaxPacket *Telemetry::exportTelemetry()
{
    return drivetrain_data; // Return our struct pointer to expose writing to telemetry data
}

/**
 * @brief Main method called in the robots periodic functions to constantly update telemetry data
 *
 */
void Telemetry::runMetrics()
{
    if (snapshot_timer->getTimer())
    {                                 // Capture a snapshot of the robots data every 100ms
        drivebase->updateTelemetry(); // Load in motor and encoder data from the drivetrain class
        captureSnapshot();            // Capture the updated drivetrain data
    }
    // Do calculations and such here
}

/**
 * @brief Create a current snapshot of the robots data and store it into a queue for later use
 *
 *
 */
void Telemetry::captureSnapshot()
{
    latest_capture = new Snapshot;
    latest_capture->left_pos = drivetrain_data->left_position;        // Average data values
    latest_capture->right_pos = drivetrain_data->right_position;      // Average data values
    latest_capture->left_speed = drivetrain_data->left_motor_power;   // Average left speed value
    latest_capture->right_speed = drivetrain_data->right_motor_power; // Average right speed value
    latest_capture->x_rot = gyro->GetYaw();                           // Get x rotation
    manageRewindBuffer();                                                   // Manage our rewind list
}

/**
 * @brief Takes data captured by the snapshot and adds it to a doubly linked list that acts as both as queue and a stack
 *
 */
void Telemetry::manageRewindBuffer()
{
    rewind_steps.push_back(latest_capture);
    if (rewind_steps.size() > max_rewind_steps)
    {
        delete rewind_steps.front(); // Free the memory allocted to the pointer
        rewind_steps.pop_front();    // Remove the snapshot pointer from the front of the list.
    }
}

// TODO: Add methodd to get the current instruction stack whether by copying or freezing new data addition so we can pass data to drivetrain input

/**
 * @brief Destroy the Telemetry:: Telemetry object
 *
 */
Telemetry::~Telemetry()
{
    delete gyro;            // Free the gyro memory
    delete drivetrain_data; // Free the telemetry struct
    delete snapshot_timer;  // Free timer
}