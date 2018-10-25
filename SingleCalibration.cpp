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
    
    cout << "Collecting images." << endl;
    cout << "Space  - to save the image's frame." << endl;
    cout << "ESC - exit the application." << endl;
    
    Mat frame; // The current frame of the camera
    
    while(true)
    {
        if(!cap.read(frame))
        {
            cout << "Error reading camera" << endl;
            continue;
        }       
        imageSize = frame.size(); // get the current size of the frame      
        imshow("Original frames", frame); // The output frame       
        int key = waitKey(30);
        if(key == 27) // ESC
        {   
            return -1;
        }
        else if(key == 32) // SPACE
        {
            cout << "Collected " << imageCount<< " images" << endl;
            string filepath = "/home/ilya/NetBeansProjects/CameraMerging/images/";
            
            ostringstream ostr; // the next three rows, convert number to string
            ostr << imageCount;
            string theNumberString = ostr.str();
            
            imwrite(filepath + theNumberString + ".jpg",frame);
            imageCount++;
        }
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
