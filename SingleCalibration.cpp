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

SingleCalibration::SingleCalibration() {
}


SingleCalibration::~SingleCalibration() {
}

int SingleCalibration::collectImages(VideoCapture& cap)
{
    if(!cap.isOpened())
    {
        cout << " Camera Checking was unsuccesfull!" << endl;
        return -1;
    }
    
    Mat edges;
    
    while(true)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        imshow("edges", edges);
        waitKey(30);
    }
    
}




int cameraCalibration()
{
    int option = 1 ; // collect images by default
       
    cout << "Choose the option:" << endl;
    cout << "1. Collect images. " << endl;
    cout << "2. Calibrate camera. " << endl;
    cout << "3. Field of view. " << endl;       
    cin >> option;     
    SingleCalibration sc;    
    switch(option) 
    {
        case 1: 
        {
            string source = "rtsp://192.168.0.162/live/main";           
            VideoCapture cap(source);         
            sc.collectImages(cap);
            break;
        }
        case 2:
        {
            break;
        }
        case 3:
        {
            break;
        }
        default:
        {
            return -1;
        }
    }  
}
