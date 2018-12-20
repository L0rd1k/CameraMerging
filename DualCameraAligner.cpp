/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DualCameraAligner.cpp
 * Author: ilya
 * 
 * Created on 21 ноября 2018 г., 11:38
 */

#include "DualCameraAligner.h"

DualCameraAligner::DualCameraAligner() {
}

DualCameraAligner::DualCameraAligner(Mat intrinsics1, Mat intrinsics2, Mat rotation)
{
    this->intrinsics1= intrinsics1;
    //cout << this->intrinsics1 << endl;
    this->intrinsics2 = intrinsics2;
    //cout << this->intrinsics2 << endl;
    this->rotation = rotation;
    //cout << this->rotation << endl;
}

Mat DualCameraAligner::align(Mat img1)
{
    
    Mat result;      
    Mat intrinsicsRotation = this->intrinsics2 * this->rotation * this->intrinsics1.inv();
    
    warpPerspective(img1, result, intrinsicsRotation, img1.size());
    imshow("TEST", result);

    return result;
}


DualCameraAligner::~DualCameraAligner() {
}

