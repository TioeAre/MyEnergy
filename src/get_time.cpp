//
// Created by Tioe on 2022/3/22.
//
#include <iostream>
#include "get_time.h"

using namespace std::chrono;
using std::cout;
using std::endl;
system_clock::duration since1970_to_now_last;
system_clock::duration since1970_to_now;
double delta;
extern bool IF_DEBUG;
extern int fps;

int get_time_() {
    system_clock::time_point time_point_now = system_clock::now();
    since1970_to_now = time_point_now.time_since_epoch();
    time_t delta_ = duration_cast<microseconds>(since1970_to_now - since1970_to_now_last).count();
    delta = (double) delta_ / 1000000;
    time_point_now = system_clock::now();
    since1970_to_now_last = time_point_now.time_since_epoch();
    fps = (int) (1 / delta);
    if (!IF_DEBUG) {
//            std::cout << "total =\t" << delta_ << " s" << std::endl << std::endl;
        cout << "fps =\t " << fps << endl;
    }
    return fps;
}

double current_time(){
    system_clock::time_point time_point_now = system_clock::now();
    system_clock::duration now;
    now = time_point_now.time_since_epoch();
    time_t delta_ = duration_cast<microseconds>(now).count();
    double time = (double) delta_ / 1000000;
    return time;
}