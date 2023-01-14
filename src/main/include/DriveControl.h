#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H
 
 #include "frc/XboxController.h"
 #include "DriveTrain.h"//Header for all of our drive control and motors

class DriveControl{

public:
DriveControl();//Ctor
~DriveControl();//Dtor
void teleopController();//Bundles drive train with other robot opertaions

private:
void tankOperation();//Enables drive train operation in tank mode
void traditionalOperation();//Enables drive train operation in single stick mode
double filterInput(double input, double threshold);//Used to filter controller input to prevent drift
void pollButtons();//Used to manage button actions on the controllers
//Members Variables
//Heap
DriveTrain* drivebase;//Object for our drive base containing the motors
frc::XboxController* controller_1;//Main Controller
frc::XboxController* controller_2;//Secondary Controller
//Stack
bool is_tank_drive;//variable to hold drive style

};





#endif //DRIVECONTROL_H