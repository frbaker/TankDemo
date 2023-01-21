#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "DriveTrain.h"


class Telemetry{

public:

Telemetry();
~Telemetry();

private:

struct SparkMaxEncoderBundle* drive_encoders;

};


#endif//TELEMETRY_H