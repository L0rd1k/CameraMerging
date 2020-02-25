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
#include <sstream>

#include "Timer.h"
#include "CalibratorBase.h"
#include "PointsCollectorBase.h"
#include "PointsCollectorChess.h"
#include "PointsCollectorCircles.h"
#include "DualCameraAligner.h"
#include "DualCameraMerger.h"

using namespace std;
using namespace cv;
typedef vector<Point2f> Points2f;
typedef vector<Point3f> Points3f;

class TwoCamerasCalibrator {
public:
    TwoCamerasCalibrator();
    virtual ~TwoCamerasCalibrator();
    int twoCamerasCalibration();
    int getPoints(vector<Points2f> *imagePoints, vector<Points3f> &objectPoints);
    int stereoCalibrateRectificate(Size boardSize, float squareSize);
    int mergeImages();
    vector<Point2f> pnts[2];
    int testHomography();
private:
    void Help();
    int loadCalibrations(int option);  
    void startCameras();
    void stopCameras();
    int collectImages();
    int grabPicture(int camera); 
    Points2f collectPoints(Mat image);
    vector<Point2d> imageResize;
    void showPoints(Mat image, Points2f& corners);
    void imageRetriever(string &folder);
private:
    string workingFold = "./DoubleCalibration/new2DoubleImages";
    Mat R, T;
    vector<string> imagelist;
    CalibratorBase* _calibrator;
};


bool isRotationMatrix(Mat &R);
Vec3f rotationMatrixToEulerAngles(Mat &R);

int loadMatrix(string filename, string name, Mat &m);
void convToGray(const Mat& in, Mat& out);
#endif /* TWOCAMERASCALIBRATOR_H */

