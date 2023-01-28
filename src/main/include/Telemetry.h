#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "DriveTrain.h"
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include "DataPacket.h"
#include "Timer.h"
#include <queue>

class Telemetry
{

public:
    Telemetry(DriveTrain *drvt);
    ~Telemetry();
    SparkMaxPacket *exportTelemetry(); // Exports the pointer to the telemetry struct
    void runMetrics();                 // Main function for updating data

private:
    void captureSnapshot();

    DriveTrain *drivebase; // Object pointer to hold reference to our drive base
    ctre::phoenix::sensors::PigeonIMU *gyro;
    SparkMaxPacket *drivetrain_data; // Holds the robot telemetry data
    Timer *snapshot_timer;           // Timer for managing snapshots of robo data
    struct Snapshot
    {
        double left_pos;
        double left_speed;
        double right_pos;
        double right_speed;
        double x_rot;
    };

    std::queue<struct Snapshot *> instruction_stack;
};

#endif // TELEMETRY_H