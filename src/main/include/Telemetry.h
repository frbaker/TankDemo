#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "DriveTrain.h"


class Telemetry{

public:

Telemetry(DriveTrain* drvt);
~Telemetry();

private:
DriveTrain* drivebase;//Object pointer to hold reference to our drive base
struct SparkMaxEncoderBundle* drive_encoders;//struct pointer to hold reference to our drive base encoders

};


#endif//TELEMETRY_H