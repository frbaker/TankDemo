#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "DriveTrain.h"
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include "DataPacket.h"

class Telemetry{

public:

Telemetry(DriveTrain* drvt);
~Telemetry();
SparkMaxPacket* exportTelemetry();//Exports the pointer to the telemetry struct
void runMetrics();//Main function for updating data

private:

DriveTrain* drivebase;//Object pointer to hold reference to our drive base
ctre::phoenix::sensors::PigeonIMU* gyro;
SparkMaxPacket* drivetrain_data;//Holds the robot telemetry data

};


#endif//TELEMETRY_H