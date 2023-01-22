#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "DriveTrain.h"
#include <ctre/phoenix/sensors/PigeonIMU.h>

class Telemetry{

public:

Telemetry(DriveTrain* drvt);
~Telemetry();
void runMetrics();//Main function for updating data

private:
void setZero();//Used to set our home/zero point when the robot starts

DriveTrain* drivebase;//Object pointer to hold reference to our drive base
ctre::phoenix::sensors::PigeonIMU* gyro;


};


#endif//TELEMETRY_H