#ifndef TIMER_H
#define TIMER_H

#include <chrono>

class Timer{

public:
    Timer(double ivl);
    ~Timer();
    bool getTimer();

private:
    void updateTimer();
    void resetTimer();
    std::chrono::time_point<std::chrono::steady_clock> current_time,previous_time;
    std::chrono::duration<std::chrono::steady_clock> duration;//Set the duration to be in milliseconds mode
    double interval;
};



#endif//TIMER_H