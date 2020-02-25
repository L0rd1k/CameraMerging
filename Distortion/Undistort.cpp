/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Undistort.cpp
 * Author: ilya
 * 
 * Created on 31 октября 2018 г., 13:49
 */

#include "Undistort.h"

Undistort::Undistort() {
}

Undistort::~Undistort() {
}

int Undistort::run() {
    Mat m, d;
    FileStorage fs;
    bool res = fs.open("../CameraMerging/SingleCalibration/TV2/intrinsics.yml", CV_STORAGE_READ);
    if(res)
    {
        fs["m"] >> m; // get matrix intrinsics coeffs
        cout << "Matrix's value: " << endl << m << endl;
        fs["d"] >> d; // get distortion coeffs
        cout << "Distortion's value: " << endl <<  d << endl;
        fs.release(); // release file storage
    } else
    {
        cout << "Cant get the intrinsics" << endl;
    }
    
    string source = "rtsp://192.168.1.168/video_1";
    VideoCapture cap(source);
    if(!cap.isOpened())
    {
        cout << " Camera Checking was unsuccesfull!" << endl;
        return -1;
    }
    
    Mat frame, undistorted, undistortedFast;
    int key = 0;
    FastUndistort fu(m,d);
    while(key != 27)
    {
        Timer tFrame;
        if(!cap.read(frame)) 
        {
            cout << "Error reading camera undistor" << endl;
            continue;
        }
        //cout << "frame time: " << tFrame.elapsedMs() << endl;
        if(tFrame.elapsedMs() > 5) {
            Timer t;    
            undistort(frame, undistorted, m, d); // Transforms an image to compensate for lens distortion.
            cout << "undistort time: " << t.elapsedMs();
            imshow("undistorted", undistorted);
            t.reset();
            
            fu.undistort(frame, undistortedFast);
            cout << ", fast undistort time: " << t.elapsedMs() << endl;
            imshow("undistortedFast", undistortedFast);
        }
        imshow("frame", frame);
        key = waitKey(1);        
    }
}

int testUndistort() 
{
    Undistort test;
    return test.run();
}