/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Undistort.h
 * Author: ilya
 *
 * Created on 31 октября 2018 г., 13:49
 */

#ifndef UNDISTORT_H
#define UNDISTORT_H

#include "FastUndistort.h"
#include "SingleCalibration.h"
#include "Timer.h"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

class Undistort {
public:
    Undistort();
    virtual ~Undistort();
    int run();
};

int testUndistort();

#endif /* UNDISTORT_H */

