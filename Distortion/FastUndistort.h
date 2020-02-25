/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastUndistort.h
 * Author: ilya
 *
 * Created on 31 октября 2018 г., 13:53
 */

#ifndef FASTUNDISTORT_H
#define FASTUNDISTORT_H

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

class FastUndistort {
public:
    FastUndistort(InputArray cameraMatrix, InputArray distCoeffs);
    virtual ~FastUndistort();
    void undistort(InputArray src, OutputArray dst);
private:
    Mat p_cameraMatrix;
    Mat p_distCoeffs;
    Mat map1;
    Mat map2;
    int p_width = 0;
    int p_height = 0;
};

#endif /* FASTUNDISTORT_H */

