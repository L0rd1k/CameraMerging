/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DualCameraAligner.h
 * Author: ilya
 *
 * Created on 21 ноября 2018 г., 11:38
 */

#ifndef DUALCAMERAALIGNER_H
#define DUALCAMERAALIGNER_H

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

class DualCameraAligner {
public:
    DualCameraAligner();
    DualCameraAligner(Mat intrinsics1, Mat instrinsics2, Mat rotation);
    Mat align(Mat img1);
    virtual ~DualCameraAligner();
private:
    Mat intrinsics1;
    Mat intrinsics2;
    Mat rotation;
};

#endif /* DUALCAMERAALIGNER_H */

