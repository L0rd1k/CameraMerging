/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FoVChecker.h
 * Author: ilya
 *
 * Created on 30 октября 2018 г., 15:53
 */

#ifndef FOVCHECKER_H
#define FOVCHECKER_H

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <string>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <array>

using namespace std;
using namespace cv;

class FoVChecker {
public:
    FoVChecker();
    FoVChecker(const FoVChecker& orig);
    virtual ~FoVChecker();
     /**
     * Calculates the angle of view for a camera on an axis
     * 
     * @param matrixIntrinsics Intrinsics matrix for camera
     * @param resolution Current resolution set in the camera
     * @param axis Either X or Y axis to calculate the Field of View. X = 0, Y = 1
     * @return Field of view in degrees
     */
    double fovCalculator(Mat matrixIntrinsics, int resolution, int axis);
     /**
     * Calculates the resolution for a camera on an axis
     * 
     * @param matrixIntrinsics Intrinsics matrix for camera
     * @param fov Field of view for a given axis
     * @param axis Either X or Y axis to calculate the Resolution. X = 0, Y = 1
     * @return Resolution in pixels
     */
    double resolutionCalculator(Mat matrixIntrinsics, double fov, int axis);
private:

};

#endif /* FOVCHECKER_H */

