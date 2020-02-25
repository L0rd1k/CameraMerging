/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SingleCalibration.h
 * Author: ilya
 *
 * Created on 25 октября 2018 г., 10:38
 */
#ifndef SINGLECALIBRATION_H
#define SINGLECALIBRATION_H

#include <opencv/cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <list>

#include "CalibratorBase.h"
#include "PointsCollectorBase.h"
#include "FoVChecker.h"

using namespace cv;
using namespace std;

class SingleCalibration {
public:
    SingleCalibration();
    virtual ~SingleCalibration();
    int collectImages(VideoCapture &cap);
    int calibrate(bool &camCheck);
    vector<Point2f> collectPoints(Mat &image, bool &camCheck);
    int calculateFoV();
    string pathCalib = "./SingleCalibration/Medium/";
protected:
    void convertToGray(const Mat& in, Mat& out);
    void showPoints(Mat &image, vector<Point2f> &corners);
    void imshow_my(const String &winname, Mat &mat);
    void calib();
    void saveCalib(Mat m, Mat d, Size imageSize);
    void openCalib(Mat& m, Mat& d, Size& imageSize);
    CalibratorBase* ptrCalibrator; // pointer to abstract class 
    Size imageSize; // the size of the frame;    
    int imageCount = 1; // the count of the frame wich was collected 
    vector< vector<Point2f> > imagePoints;
};

int cameraCalibration();

#endif /* SINGLECALIBRATION_H */

