/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SingleCalibration.cpp
 * Author: ilya
 * 
 * Created on 25 октября 2018 г., 10:38
 */


#include "SingleCalibration.h"
#include "PointsCollectorChess.h"
#include "PointsCollectorCircles.h"

SingleCalibration::SingleCalibration() {
    ptrCalibrator = new PointsCollectorChess(9, 6, 25);
    //ptrCalibrator = new PointsCollectorCircles(4,11,39);
}

SingleCalibration::~SingleCalibration() {
    delete ptrCalibrator;
}

int SingleCalibration::collectImages(VideoCapture& cap) {
    if (!cap.isOpened()) {
        cout << "Camera Checking was unsuccesfull!" << endl;
        return -1;
    }
    cout << "Collecting image........" << endl;
    cout << "       Space  - to save the image's frame." << endl;
    cout << "       ESC - exit the application." << endl;
    Mat frame;
    while (true) {
        if (!cap.read(frame)) {
            cout << "Error reading camera collect" << endl;
            continue;
        }
#ifndef IMG_SIZE
        imageSize = frame.size();
        cout << imageSize << endl;
#endif   
        cvtColor(frame, frame, CV_BGR2GRAY);
        Mat image = frame;
        vector<Point2f> pt;
#define CIRCLE_CALIB
#ifndef CIRCLE_CALIB
        int flags = CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING;
        bool circlesResult = findCirclesGrid(image, ptrCalibrator->getChessboardSize(), pt, flags);
#endif
        int flags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK;
        bool circlesResult = findChessboardCorners(image, ptrCalibrator->getChessboardSize(), pt, flags);
        Mat cornersImage;
        cvtColor(image,cornersImage, CV_GRAY2BGR);
        int key = waitKey(1);
        if (circlesResult) {
            drawChessboardCorners(cornersImage, ptrCalibrator->getChessboardSize(), pt, true); // draw a specific points on the image 
            imshow("Corners", cornersImage);
            pt.clear();
            if (key == 'x' || key == 120) {
                ostringstream ostr;
                ostr << imageCount;
                string theNumberString = ostr.str();
                imwrite(pathCalib + theNumberString + ".jpg", image);
                imageCount++;
            }
        } else {
            imshow("Corners", image);
        }
        if (key == 27)
            return -1;
        waitKey(1);
    }
    return imageCount;
}

vector<Point2f> SingleCalibration::collectPoints(Mat &image, bool &camCheck) {
    imageSize = image.size(); // image size(for instance 1356x1024) 
    cout << imageSize << endl;
    vector<Point2f> currentPoints; // Points from the given image 
    Mat gray;
    convertToGray(image, gray); // convert original frame to gray
//#define IK_BITWISE
//#ifndef IK_BITWISE
    if (camCheck) {
        bitwise_not(gray, gray); // ! ONLY FOR IK CAMERAS ! // inverts every bit of an array
    }
//#endif
    currentPoints = ptrCalibrator->collectFramePoints(gray); // here we got specific corners points from the image
    imagePoints.push_back(currentPoints); // every points (x,y)
    cout << "size:" << image.size() << endl;
    showPoints(gray, currentPoints); //Show points ont he image
    return currentPoints;
}

void SingleCalibration::showPoints(Mat &image, vector<Point2f> & corners) {
    if (corners.size() > 0) { 
        Mat cornersImage;
        cvtColor(image, cornersImage, CV_GRAY2BGR); // convert To RGB again
        drawChessboardCorners(cornersImage, ptrCalibrator->getChessboardSize(), corners, true); // draw a specific points on the image 
        imshow_my("Corners", cornersImage);
    } else {
        imshow_my("Corners", image);
        cout << "No points found!" << endl;
    }
    waitKey(1);
}

void SingleCalibration::imshow_my(const String& winname, Mat& mat) {
    Size maxSize(1600, 1200);
    Mat resized;
    // resize image if it's large then 1600x1200 
    if (mat.cols > maxSize.width || mat.rows > maxSize.height) {
        resize(mat, resized, maxSize);
        imshow(winname, resized);
    } else {
        imshow(winname, mat);
    }
}

int SingleCalibration::calibrate(bool &camCheck) {
    cout << "The process of calibration..." << endl;
    string folder = pathCalib + "*.jpg";
    vector<String> filename; // vector for saving all files which we found in folder
    glob(folder, filename); // the function for searching specific files
    cout << "The numbers of frames " << filename.size() << endl;
    for (int i = 0; i < filename.size(); i++) {
        cout << "Loading " << i + 1 << "........." << filename[i] << endl;
        Mat image = imread(filename[i]); // read the files from the vector
        cout << "CAM CHECK =  " << camCheck;
        collectPoints(image, camCheck); // main function for collecting point from a given image
    }
    calib();
    return 0;
}

void SingleCalibration::calib() {
    vector< vector<Point3f> > objectPoints;
    int samplesCounter = imagePoints.size(); // 25 Points (x,y)
    for (int i = 0; i < samplesCounter; ++i) {
        objectPoints.push_back(ptrCalibrator->collectObjectPoints()); //from (x,y) to (x,y,z)  
    }
    Mat m = Mat::eye(3, 3, CV_64F);
    Mat d = Mat::zeros(5, 1, CV_64F); 
    vector<Mat> t, r; // rotation and translation vector
    cout << "Image Points Size : " << imagePoints.size() << endl;
    cout << objectPoints.size() << endl;
    cout << imagePoints.size() << endl;
    cout << imageSize << endl;
    double err = calibrateCamera(objectPoints, imagePoints, imageSize, m, d, r, t,
            CV_CALIB_FIX_K4 |
            CV_CALIB_FIX_K5,
            TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));
    cout << "Calibration error: " << err << endl; // At the end we got Pre-error of calibration
    saveCalib(m, d, imageSize);
}

void SingleCalibration::saveCalib(Mat m, Mat d, Size imageSize) {
    string filename = pathCalib + "intrinsics.yml";
    FileStorage fs(filename, CV_STORAGE_WRITE);
    if (fs.isOpened()) {
        fs << "m" << m << "d" << d << "s" << imageSize;
        fs.release();
    }
    cout << "Saved calibration data to " << filename << endl;
}

void SingleCalibration::convertToGray(const Mat& in, Mat& out) {
    if (in.channels() == 3)
        cvtColor(in, out, CV_BGR2GRAY);
    else
        in.copyTo(out);
}


int SingleCalibration::calculateFoV() {
    Mat m, d;
    Size imageSize;
    openCalib(m, d, imageSize);
    FoVChecker fovChecker;
    
    double x = fovChecker.fovCalculator(m, imageSize.width, 0);
    double y = fovChecker.fovCalculator(m, imageSize.height, 1);

    double xRes = fovChecker.resolutionCalculator(m, x, 0);
    double yRes = fovChecker.resolutionCalculator(m, y, 0);
    cout << "xRes: " << xRes << " yRes: " << yRes << endl;

    cout << "Degress in x axis: " << x << endl;
    cout << "Degress in y axis: " << y << endl;
    return 0;
}

void SingleCalibration::openCalib(Mat& m, Mat& d, Size& s) {
    string filename = pathCalib + "intrinsics.yml";
    FileStorage fs(filename, CV_STORAGE_READ);
    if (fs.isOpened()) {
        fs["m"] >> m;
        fs["d"] >> d;
        fs["s"] >> s;
        fs.release();
    }
}

int cameraCalibration() {
    int option; // collect images by default 
    
    cout << "1. Collect images. " << endl;
    cout << "2. Calibrate cameras. " << endl;
    cout << "3. Field of view. " << endl;
    cout << "Choose the option: " << endl;
    
    SingleCalibration sc;
    cin >> option;
    switch(option) {
        case 1: {
            string source = "rtsp://192.168.1.168/video_1";
            //string source = "rtsp://192.168.1.168/video_1";
            VideoCapture cap(source);
            return sc.collectImages(cap);
            break;
        }
        case 2: {
            cout << "1 - Calibration for TV camera.  " << endl;
            cout << "2 - Calibration for IK camera.  " << endl;
            int camOption;
            bool camCheck = false;
            cin >> camOption;
            camCheck = (camOption == 1) ? false : true; // check for IK (bitwise param)
            return sc.calibrate(camCheck);
            break;
        }
        case 3: {
            sc.calculateFoV();
            break;
        }
        default: {
            return -1;
        }
    }
}
