/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TwoCamerasCalibrator.cpp
 * Author: ilya
 * 
 * Created on 7 ноября 2018 г., 12:27
 */

#include "TwoCamerasCalibrator.h"

#include <sys/stat.h>

#define MAX_CAMERA_RESOLUTION 10000
#define FPS 25
#define NUM_IMAGES  30
#define BOARD_SIZE_WIDHT    4
#define BOARD_SIZE_HEIGHT   11

unique_ptr<std::thread> cam0_ptr;
unique_ptr<std::thread> cam1_ptr;
atomic<bool> threadRunning(true);
atomic<bool> hasFrame0(false);
atomic<bool> hasFrame1(false);
mutex m0,m1;

TwoCamerasCalibrator::TwoCamerasCalibrator() 
{
}

TwoCamerasCalibrator::~TwoCamerasCalibrator() 
{
    
}

int loadMatrix(string filename, string name, Mat &m)
{
    FileStorage fs;
    int result = fs.open(filename, CV_STORAGE_READ); // checking for opening file
    if (result)
    {
        fs[name] >> m;
        fs.release();
        if(m.rows > 0)
        {
            return EXIT_SUCCESS; // return EXIT_SUCCESS;
        } else
        {
            cerr << "Matrix " << name << " was empty in file " << filename << endl;     
        }
    } 
    else 
    {
        cerr << "Matrix " << name << " not found in file" << filename << endl;
    }
    return EXIT_FAILURE; // return EXIT_FAILURE;
}

int TwoCamerasCalibrator::loadCalibrations(int option)
{
    if(loadMatrix("/home/ilya/NetBeansProjects/CameraMerging/InfImagesBest/intrinsics.yml","m",intrinsicsMatrix[0]))
    {
        return 1;
    }
    if(loadMatrix("/home/ilya/NetBeansProjects/CameraMerging/InfImagesBest/intrinsics.yml","d",distortions[0]))
    {
        return 1;
    }
       
    if(loadMatrix("/home/ilya/NetBeansProjects/CameraMerging/images/intrinsics.yml","m", intrinsicsMatrix[1]))
    {
        return 1;
    }
    if(loadMatrix("/home/ilya/NetBeansProjects/CameraMerging/images/intrinsics.yml","d", distortions[1]))
    {
        return 1;
    }
    
    
    return EXIT_SUCCESS; // return EXIT_SUCCESS;
}

int TwoCamerasCalibrator::grabPicture(VideoCapture cap, int camera) 
{
    Mat frameCamDistorted[2];
    if(!cap.isOpened())
    {
        return EXIT_FAILURE;
    }
    while(threadRunning)
    {
        Timer tFrame;
        cap.grab(); // grab next frame of the video
        if(camera == 0)
        {
            cap.retrieve(frameCamDistorted[0]);
            if(frameCamDistorted[0].size().height > 0)
            {
                lock_guard<mutex> guard(m1);
                frameCam[0] = frameCamDistorted[0];
                hasFrame0 = true;
            }
        }
        else
        {
            cap.retrieve(frameCamDistorted[1]);
            if(frameCamDistorted[1].size().height > 0)
            {
                lock_guard<mutex> guard(m1);
                frameCam[1] = frameCamDistorted[1];
                hasFrame1 = true;
            }
        }
    }
    return EXIT_SUCCESS;
}


void TwoCamerasCalibrator::startCameras()
{
    // FIRST CAMERA
    VideoCapture cap0("rtsp://192.168.0.162/live/main");
    cap0.set(CV_CAP_PROP_FRAME_WIDTH, MAX_CAMERA_RESOLUTION);
    cap0.set(CV_CAP_PROP_FRAME_HEIGHT, MAX_CAMERA_RESOLUTION);
    resolutionCamX[0] = static_cast<int>(cap0.get(CV_CAP_PROP_FRAME_WIDTH));
    resolutionCamY[0] = static_cast<int>(cap0.get(CV_CAP_PROP_FRAME_HEIGHT));
    cap0.set(CV_CAP_PROP_FPS, FPS);
       
    // SECOND CAMERA
    VideoCapture cap1("rtsp://192.168.1.168/video_1");
    cap1.set(CV_CAP_PROP_FRAME_WIDTH, MAX_CAMERA_RESOLUTION);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT, MAX_CAMERA_RESOLUTION);
    resolutionCamX[1] = static_cast<int>(cap1.get(CV_CAP_PROP_FRAME_WIDTH));
    resolutionCamY[1] = static_cast<int>(cap1.get(CV_CAP_PROP_FRAME_HEIGHT));
    cap1.set(CV_CAP_PROP_FPS, FPS);
       
    cam0_ptr = unique_ptr<std::thread>(new thread(&TwoCamerasCalibrator::grabPicture, this, cap0, 0));
    cam1_ptr = unique_ptr<std::thread>(new thread(&TwoCamerasCalibrator::grabPicture, this, cap1, 1));
            
}
void TwoCamerasCalibrator::stopCameras()
{
    threadRunning = false;
    cam0_ptr->join(); // wait until second camera ended its job
    cam1_ptr->join(); // wait until first camera ended its job
}

int TwoCamerasCalibrator::collectImages()
{
    Mat frameCamCopy[2];
    mkdir("/home/ilya/NetBeansProjects/CameraMerging/DoubleImages", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cout << "Press 'x' to capture a new image" << endl;
    int key = 0;
    int count = 1;
    while(key != 27)
    {       
        
        if(hasFrame0)
        {
            lock_guard<mutex> guard1(m0);
            frameCamCopy[0] = frameCam[0];
            cout << frameCamCopy[0].size() << endl;
            imshow("Frame Camera [0]", frameCamCopy[0]);
            hasFrame0 = false;
        }
        
        if(hasFrame1)
        {
            lock_guard<mutex> guard2(m1);
            frameCamCopy[1] = frameCam[1];
            cout << frameCamCopy[1].size() << endl;
            imshow("Frame Camera [1]", frameCamCopy[1]);
            hasFrame1 = false;
        }
        

        if(key == 'x')
        {
            string name1 = "/home/ilya/NetBeansProjects/CameraMerging/DoubleImages/cam_" + to_string(count++) + ".jpg";
            imagelist.push_back(name1);
            string name2 = "/home/ilya/NetBeansProjects/CameraMerging/DoubleImages/cam_" + to_string(count++) + ".jpg";
            imagelist.push_back(name2);
            lock_guard<mutex> guard1(m0);
            imwrite(name1, frameCam[0]);
            lock_guard<mutex> guard2(m1);
            imwrite(name2, frameCam[1]);
            
            cout << "Press 'x' to capture a new image" << endl;
            
            if(count >= NUM_IMAGES * 2)
                break;
        }
        key = waitKey(10);
    }
    return EXIT_SUCCESS; // EXIT_SUCCESS
}




int TwoCamerasCalibrator::twoCamerasCalibration()
{
    int option = 1 ; // collect images by default       
    cout << "1. Collect images for StereoCalibration " << endl;
    cout << "2. " << endl;
    cout << "3. " << endl; 
    cout << "Choose the option: ";
    cin >> option;   
    
    //Checking if the yml. files both cameras was added
    if(loadCalibrations(option) == EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
      
    switch (option)
    {
        case 1: 
        {
            startCameras();
            collectImages();
            stopCameras();
            break;
        }
        case 2:
        {
            Size boardSize;
            boardSize.width = BOARD_SIZE_WIDHT;
            boardSize.height = BOARD_SIZE_HEIGHT;
            break;
        }
        case 3:
        {
            break;
        }
        default:
            return 1; // return EXIT_FAILURE
            
    }
    return 0; // EXIT_SUCCESS
}