#ifndef ENCODERS_H
#define ENCODERS_H

#include <rev/SparkMaxRelativeEncoder.h>

//Holds four spark max motor encoders
struct SparkMaxEncoderBundle{
    rev::SparkMaxRelativeEncoder* m1;
    rev::SparkMaxRelativeEncoder* m2;
    rev::SparkMaxRelativeEncoder* m3;
    rev::SparkMaxRelativeEncoder* m4;

};



#endif//ENCODERS_H