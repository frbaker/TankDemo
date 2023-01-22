#ifndef DATA_PACKET_H
#define DATA_PACKET_H

struct SparkMaxPacket{
    double motor_power_1;
    double motor_power_2;
    double motor_power_3;
    double motor_power_4;
    double encoder_position_1;
    double encoder_position_2;
    double encoder_position_3;
    double encoder_position_4;
};



#endif//DATA_PACKET_H