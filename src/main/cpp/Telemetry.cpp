#include "Telemetry.h"


Telemetry::Telemetry(DriveTrain* drvt){
drivebase = drvt;//Reference our created drivetrain
gyro = new ctre::phoenix::sensors::PigeonIMU(16);//Create our gyro object with can ID 16
setZero();//Set our home positions
}

void Telemetry::setZero(){
    //Set motors to zero revolution
    //drive_encoders->motor_encoder_1->SetPosition(0.0);
    //drive_encoders->motor_encoder_2->SetPosition(0.0);
    //drive_encoders->motor_encoder_3->SetPosition(0.0);
    //drive_encoders->motor_encoder_4->SetPosition(0.0);
    gyro->SetYaw(0.0);//Set the x angle to 0
}

void Telemetry::runMetrics(){

}


Telemetry::~Telemetry(){
    delete gyro;//Free the gyro memory
}