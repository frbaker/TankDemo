#include "Telemetry.h"


Telemetry::Telemetry(DriveTrain* drvt){
drivebase = drvt;//Reference our created drivetrain
drive_encoders = drivebase->getEncoders();//Get our encoder data
}

Telemetry::~Telemetry(){
//No freeing, since we are not allocating any new memory here
}