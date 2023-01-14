#include "Timer.h"

Timer::Timer(double ivl){
    interval = ivl;
    //Start Timer
    current_time = std::chrono::steady_clock::now();
    previous_time = current_time;

}


bool Timer::getTimer(){
    updateTimer();
    if(std::chrono::duration_cast<std::chrono::milliseconds>(current_time-previous_time).count() >= interval){//If we reached the interval
        resetTimer();//Resetart the timer
        return true;//Return true if the timer was reached
    }
    return false;//Return that the timer has not been reached
}

void Timer::updateTimer(){
    current_time = std::chrono::steady_clock::now();
}

void Timer::resetTimer(){
current_time = std::chrono::steady_clock::now();//Might not actually need this line
previous_time = current_time;//Set previous time to be the same as current time, so time difference is reset
}

Timer::~Timer(){
    //Dtor
}