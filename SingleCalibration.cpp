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

SingleCalibration::SingleCalibration() 
{
    ptrCalibrator = new PointsCollectorChess(9,6,25);
}


SingleCalibration::~SingleCalibration()
{
    
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
    return imageCount;
}

vector<Point2f> SingleCalibration::collectPoints(Mat image)
{
    imageSize = image.size();
    
    cout << imageSize << endl;
    
    vector<Point2f> currentPoints; 
    
    const int maxScale = 1;  
    Mat gray;  
    convertToGray(image,gray); // convert original frame to gray
    //bitwise_not(gray, gray);   // inverts every bit of an array.
    
    
    
    for(int s = 1; s <= maxScale; s++)
    {
        Mat tmpImage = gray;
        if(s > 1)
        {
            resize(gray, tmpImage, Size(0,0), s, s, CV_INTER_CUBIC);
        }       
        
        currentPoints = ptrCalibrator->collectFramePoints(tmpImage);
        
//        cout << "Current Points" << currentPoints << endl;
        
        //cout << currentPoints.size() << endl;
        
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
    
    showPoints(gray,currentPoints);
    
    return currentPoints;
}

void SingleCalibration::showPoints(Mat image, vector<Point2f> & corners)
{
    if(corners.size() > 0)
    {
        Mat cornersImage;
        cvtColor(image,cornersImage, CV_GRAY2BGR);
        drawChessboardCorners(cornersImage, ptrCalibrator->getChessboardSize(),corners,true);
        imshow_my("Corners", cornersImage);
        waitKey(10000);
    }
}

void SingleCalibration::imshow_my(const String& winname, Mat& mat)
{
    Size maxSize(1600,1200);
    Mat resized;
    if(mat.cols > maxSize.width || mat.rows > maxSize.height) 
    {
        resize(mat, resized, maxSize);
        imshow(winname, resized);
    } else 
    {
        imshow(winname, mat);
    }
}

int SingleCalibration::calibrate()
{
    cout << "The process of calibration" << endl;
    string folder = "/home/ilya/NetBeansProjects/CameraMerging/images/*.jpg";
    vector<String> filename; 
    glob(folder,filename);
    cout << "The numbers of frames " << filename.size() << endl; 
    
    for(int i = 0; i < filename.size(); i++)
    {
        cout << "Loading " << i+1 <<"........."  << filename[i] << endl;
        Mat image = imread(filename[i]);
        //cout << image << endl;
        collectPoints(image);
    }
    calib();  
    return 0;
}


void SingleCalibration::calib()
{
    vector< vector<Point3f> > objectPoints;
    int samplesCounter = imagePoints.size();
    for (int i=0; i < samplesCounter; ++i)
    {
        objectPoints.push_back(ptrCalibrator->collectObjectPoints());    
    }
    
    Mat m,d;
    vector <Mat> t,r;
     
    cout << imagePoints.size() << endl;
    double err = calibrateCamera(objectPoints, imagePoints, imageSize, m, d, r, t,
                                    CV_CALIB_FIX_K4 |
                                    CV_CALIB_FIX_K5 |
                                    CV_CALIB_FIX_K6 |
                                    CV_CALIB_FIX_K3 |
                                    CV_CALIB_ZERO_TANGENT_DIST,
                                    TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));
    cout << "Calibration error" << err << endl;
    saveCalib(m,d, imageSize);
}

void SingleCalibration::saveCalib(Mat m, Mat d, Size imageSize)
{
    const char* filename = "/home/ilya/NetBeansProjects/CameraMerging/images/intrinsics.yml";
    FileStorage fs(filename, CV_STORAGE_WRITE);
    if(fs.isOpened())
    {
        fs << "m" << m << "d" << d << "s" << imageSize;
        fs.release();
    }
    cout << "Saved calibration data to " << filename << endl;
}



void SingleCalibration::convertToGray(const Mat& in, Mat& out)
{
    if(in.channels() == 3)
        cvtColor(in, out, CV_BGR2GRAY);
    else
        in.copyTo(out);
}

int SingleCalibration::calculateFoV()
{
    Mat m,d;
    Size imageSize;
    openCalib(m, d, imageSize);
    FoVChecker fovChecker;
    double x = fovChecker.fovCalculator(m, imageSize.width, 0);
    double y = fovChecker.fovCalculator(m, imageSize.height, 1);
    
    cout << "Degress in x axis: " << x << endl;
    cout << "Degress in y axis: " << y  << endl;  
        
    return 0;
}

void SingleCalibration::openCalib(Mat& m, Mat& d, Size& s)
{
    const char* filename = "/home/ilya/NetBeansProjects/CameraMerging/images/intrinsics.yml";
    FileStorage fs(filename, CV_STORAGE_READ);
    if (fs.isOpened()) {
        fs["m"] >> m;
        fs["d"] >> d;
        fs["s"] >> s;
        fs.release();
    } 
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
            sc.calculateFoV();
            break;
        }
        default:
        {
            return -1;
        }
    }  
}
