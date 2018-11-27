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
#include <string>

#include <pthread.h>
#include <thread>
#include <atomic>
#include <mutex>

#include "Timer.h"
#include "CalibratorBase.h"
#include "PointsCollectorBase.h"
#include "PointsCollectorChess.h"
#include "PointsCollectorCircles.h"
#include "DualCameraAligner.h"
#include "DualCameraMerger.h"

using namespace std;
using namespace cv;

class TwoCamerasCalibrator {
public:
    TwoCamerasCalibrator();
    virtual ~TwoCamerasCalibrator();
    int twoCamerasCalibration();
    int stereoCalibrateRectificate(const vector<string>& imagelist, Size boardSize, float squareSize);
    int mergeImages();
private:
    int loadCalibrations(int option);  
    void startCameras();
    void stopCameras();
    int collectImages();
    int grabPicture(int camValue,VideoCapture cap, int camera); 
    vector<Point2f> collectPoints(Mat image);
    vector<Point2d> imageResize;
    void showPoints(Mat image, vector<Point2f>& corners);
private:
    Mat R, T;
    Mat intrinsicsMatrix[2], distortions[2];
    Mat intrinsicsMatrixUndistort[2], distortionsUndistort[2];
    Mat frameCam[2];
    double resolutionCamX[2];
    double resolutionCamY[2]; 
    vector<string> imagelist;
    CalibratorBase* _calibrator;
};

int loadMatrix(string filename, string name, Mat &m);
void convToGray(const Mat& in, Mat& out);
#endif /* TWOCAMERASCALIBRATOR_H */

