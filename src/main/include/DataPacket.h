#ifndef DATA_PACKET_H
#define DATA_PACKET_H

struct SparkMaxPacket
{
    double left_motor_power;
    double right_motor_power;
    double left_position;
    double right_position;
};

#endif // DATA_PACKET_H