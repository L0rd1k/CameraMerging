/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Timer.h
 * Author: ilya
 *
 * Created on 31 октября 2018 г., 15:14
 */

#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <ctime>
#include <assert.h>
#include <string>
#include <sstream>
#include <iostream>
using namespace std;

class Timer {
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const 
    {
        return chrono::duration_cast<second_> (clock_::now() - beg_).count(); 
    }
    double elapsedMs() const 
    {
        return elapsed() * 1000;
    }

private:
    typedef chrono::high_resolution_clock clock_;
    typedef chrono::duration<double, ratio<1> > second_; chrono::time_point<clock_> beg_;
};

#endif /* TIMER_H */

