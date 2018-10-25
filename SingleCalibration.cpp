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
#include "CalibratorBase.h"

SingleCalibration::SingleCalibration() 
{
    ptrCalibrator = new PointsCollectorCircles();
    ptrCalibrator = new PointsCollectorChess();
}


SingleCalibration::~SingleCalibration(){
    
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
            
            ostringstream ostr; // the next three rows, convert number to string
            ostr << imageCount;
            string theNumberString = ostr.str();
            
            string filepath = "/home/ilya/NetBeansProjects/CameraMerging/images/";
            imwrite(filepath + theNumberString + ".jpg",frame);
            imageCount++;
        }
    }
    
}

vector<Point2f> SingleCalibration::collectPoints(Mat image)
{
    imageSize = image.size();
    cout << imageSize << endl;
    vector<Point2f> currentPoints;
    const int maxScale = 1;  
    Mat gray;  
    convertToGray(image,gray); // convert original frame to gray
    bitwise_not(gray, gray);   // inverts every bit of an array.
    for(int s = 1; s <= maxScale; s++)
    {
        Mat tmpImage = gray;
        if(s > 1)
        {
            resize(gray, tmpImage, Size(0,0), s, s, CV_INTER_CUBIC);
        }       
        
        currentPoints = ptrCalibrator->collectFramePoints(tmpImage);
        cout << "Current Points" << currentPoints << endl;
        if( currentPoints.size() > 0)
        {
            for(uint j=0; j < currentPoints.size(); j++)
            {
                currentPoints[j].x = currentPoints[j].x / s;
                currentPoints[j].y = currentPoints[j].y / s;
            }
            imagePoints.push_back(currentPoints);
            break;
        }
    }   
    //showPoints(gray,currentPoints);
    return currentPoints;
}

void SingleCalibration::calibrate()
{
    cout << "The process of calibration" << endl;
    string folder = "/home/ilya/NetBeansProjects/CameraMerging/images/*.jpg";
    vector<String> filename; 
    glob(folder,filename);
    cout << "The numbers of frames " << filename.size() << endl; 
    for(int i = 0; i < filename.size(); i++)
    {
        cout << "Loading........." << filename[i] << endl;
        Mat image = imread(filename[i]);
        //cout << image << endl;
        collectPoints(image);
    }
}


void SingleCalibration::convertToGray(const Mat& in, Mat& out)
{
    if(in.channels() == 3)
        cvtColor(in, out, CV_BGR2GRAY);
    else
        in.copyTo(out);
}


int cameraCalibration()
{
    int option = 1 ; // collect images by default
       
    cout << "1. Collect images. " << endl;
    cout << "2. Calibrate camera. " << endl;
    cout << "3. Field of view. " << endl; 
    cout << "Choose the option: ";
    cin >> option;     
    SingleCalibration sc;    
    switch(option) 
    {
        case 1: 
        {
            string source = "rtsp://192.168.0.162/live/main";           
            VideoCapture cap(source);         
            return sc.collectImages(cap);
            break;
        }
        case 2:
        {
            return sc.calibrate();
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
