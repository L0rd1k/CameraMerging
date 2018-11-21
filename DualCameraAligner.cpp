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
    this->intrinsics2 = intrinsics2;
    this->rotation = rotation;
}

Mat DualCameraAligner::align(Mat img1)
{
    Mat result;
    
    Mat intrinsicsRotation = this->intrinsics2 * this->rotation * this->intrinsics1.inv();
    warpPerspective(img1, result, intrinsicsRotation, img1.size());
    return result;
}


DualCameraAligner::~DualCameraAligner() {
}

