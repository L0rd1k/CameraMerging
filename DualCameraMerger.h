/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DualCameraMerger.h
 * Author: ilya
 *
 * Created on 21 ноября 2018 г., 11:39
 */

#ifndef DUALCAMERAMERGER_H
#define DUALCAMERAMERGER_H

#include "opencv/cv.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class DualCameraMerger {
public:
    DualCameraMerger();
    Mat merge(Mat img1,Mat img2);
    virtual ~DualCameraMerger();
};

#endif /* DUALCAMERAMERGER_H */

