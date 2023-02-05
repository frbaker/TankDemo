#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "DriveTrain.h"
#include "RobotAuxilary.h"
#include "Timer.h"
#include <list>

struct Snapshot
    {
        double left_pos;
        double left_speed;
        double right_pos;
        double right_speed;
        double x_rot;
    };


class Telemetry
{

public:
    Telemetry(DriveTrain *drvtobj, RobotAuxilary* auxobj);
    ~Telemetry();
    void runMetrics();                 // Main function for updating data
    Snapshot* exportSnapshot();//Export captured snapshot data

private:
    void captureSnapshot();
    void manageRewindBuffer();
    DriveTrain *drivebase; // Object pointer to hold reference to our drive base
    RobotAuxilary* utilities;//Object pointer to hold refernce to our robot utilities
    Timer *snapshot_timer;           // Timer for managing snapshots of robo data
    unsigned int timer_interval;     // How often we should take robot snapshots

    struct Snapshot *latest_capture; // Holds the robot data from the latest snapshot
    std::list<struct Snapshot *> rewind_steps;
    unsigned int max_rewind_time;  // Sets how long we store data of previous positioning
    unsigned int max_rewind_steps; // Changes depending on poll time to store 5s of past position data
};

#endif // TELEMETRY_H