#include "DriveTrain.h"

//Ctor
DriveTrain::DriveTrain(){
    //Spark motor controller creation for neo brushless motors
    left_motor_1 = new rev::CANSparkMax(4,rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    left_motor_2 = new rev::CANSparkMax(5,rev::CANSparkMax::MotorType::kBrushless);
    right_motor_1 = new rev::CANSparkMax(6,rev::CANSparkMax::MotorType::kBrushless);
    right_motor_2 = new rev::CANSparkMax(7,rev::CANSparkMax::MotorType::kBrushless);

    //We have to create a copy of the object created by calling get encoder, which returns an object, not a pointer
    left_encoder_1 = new rev::SparkMaxRelativeEncoder(left_motor_1->GetEncoder());
    left_encoder_2 = new rev::SparkMaxRelativeEncoder(left_motor_2->GetEncoder());
    right_encoder_1 = new rev::SparkMaxRelativeEncoder(right_motor_1->GetEncoder());
    right_encoder_2 = new rev::SparkMaxRelativeEncoder(right_motor_2->GetEncoder());

    left_motor_1->SetInverted(true);//Invert left side motors since they are flipped
    left_motor_2->SetInverted(true);//Invert left side motors since they are flipped

    left_motor_2->Follow(*left_motor_1);//Have to dereference pointer since function is defined as pass by reference, so we need to
    right_motor_2->Follow(*right_motor_1);//pass the address of the object, not the address of the pointer

    /*TalonFX Motor Controllers
    leftmotor1 = new ctre::phoenixpro::hardware::TalonFX(0,"");//Leave can name empty for auto init
    leftmotor2 = new ctre::phoenixpro::hardware::TalonFX(1,"");//Leave can name empty for auto init
    rightmotor1 = new ctre::phoenixpro::hardware::TalonFX(2,"");//Leave can name empty for auto init
    rightmotor2 = new ctre::phoenixpro::hardware::TalonFX(3,"");//Leave can name empty for auto init
    */
}


void DriveTrain::setSpeed(double ls, double rs){
    //Set the speeds for each side of the robots drive train
    left_motor_1->Set(ls);
    //leftmotor2->Set(ls);//Should be controlled by motor 1
    right_motor_1->Set(rs);
    //rightmotor2->Set(rs);//Should be controlled by motor 1
}

void DriveTrain::getSpeeds(){
    left_motor_1->Get();
}


DriveTrain::~DriveTrain(){
    //Motors
    delete left_motor_1;
    delete left_motor_2;
    delete right_motor_1;
    delete right_motor_2;
    //Encoders
    delete left_encoder_1;
    delete left_encoder_2;
    delete right_encoder_1;
    delete right_encoder_2;
}