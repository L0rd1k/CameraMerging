/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TwoCamerasCalibrator.h
 * Author: ilya
 *
 * Created on 7 ноября 2018 г., 12:27
 */

#ifndef TWOCAMERASCALIBRATOR_H
#define TWOCAMERASCALIBRATOR_H

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

class TwoCamerasCalibrator {
public:
    TwoCamerasCalibrator();
    virtual ~TwoCamerasCalibrator();
private:
protected:
    

};

int twoCamerasCalibration();

#endif /* TWOCAMERASCALIBRATOR_H */

