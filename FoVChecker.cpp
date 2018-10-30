/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FoVChecker.cpp
 * Author: ilya
 * 
 * Created on 30 октября 2018 г., 15:53
 */

#include "FoVChecker.h"

FoVChecker::FoVChecker() {
}

FoVChecker::FoVChecker(const FoVChecker& orig) {
}

FoVChecker::~FoVChecker() {
}

double FoVChecker::fovCalculator(Mat matrixIntrinsics, int resolution, int axis) 
{
    double focal_distance = matrixIntrinsics.at<double>(axis, axis);
    return 2 * atan(resolution / (2 * focal_distance)) * (180.0 / 3.141592653589793238463);


}

double FoVChecker::resolutionCalculator(Mat matrixIntrinsics, double fov, int axis) 
{
    double fov_rad = fov * 3.141592653589793238463 / 180;
    return 2 * matrixIntrinsics.at<double>(axis, axis) * tan(fov_rad / 2);
}

