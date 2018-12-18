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
    //ptrCalibrator = new PointsCollectorChess(9,6,25);
    ptrCalibrator = new PointsCollectorCircles(4,11,44);
}

SingleCalibration::~SingleCalibration()
{
    delete ptrCalibrator; 
}

int SingleCalibration::collectImages(VideoCapture& cap)
{
    if(!cap.isOpened())
    {
        cout << " Camera Checking cwas unsuccesfull!" << endl;
        return -1;
    }
    
    cout << "Collecting image........" << endl;
    cout << "       Space  - to save the image's frame." << endl;
    cout << "       ESC - exit the application." << endl;
    
    Mat frame; 
    
    // The current frame of the camera
  
    while(true)
    {
        if(!cap.read(frame))
        {
            cout << "Error reading camera" << endl;
            continue;
        }       
        
        imageSize = frame.size();
        cout << imageSize << endl;
        
        cvtColor(frame,frame,CV_BGR2GRAY);
        //bitwise_not(frame, frame); 
        Mat image = frame;
        vector<Point2f> pt;
        int flags;
        flags = CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING;
        bool circlesResult = findCirclesGrid(image, ptrCalibrator->getChessboardSize(), pt, flags);
        Mat cornersImage;
        cvtColor(image,cornersImage, CV_GRAY2BGR);
        if(circlesResult)
        {
            drawChessboardCorners(cornersImage,ptrCalibrator->getChessboardSize(),pt,true); // draw a specific points on the image 
            imshow("Corners", cornersImage); 
            pt.clear();
            
            int key = waitKey(1);
            if(key == 27)
            { 
                return -1;
            }
            else if(key == 32)
            {
                ostringstream ostr;
                ostr << imageCount;
                string theNumberString = ostr.str();
                string filepath = "./IK/";
                imwrite(filepath + theNumberString + ".jpg", cornersImage); // writing an image to folder
                imageCount++;
            }    
        } else 
        {
            imshow("Corners",image);
        }
        waitKey(1);
    }
    
//    while(true)
//    {                
//        if(!cap.read(frame))
//        {
//            cout << "Error reading camera" << endl;
//            continue;
//        }      
//        
//        imageSize = frame.size(); // get the current size of the frame 
//        cout << imageSize << endl;
//        imshow("Original frames", frame); // The output frame       
//        int key = waitKey(30);
//        if(key == 27) // ESC
//        {   
//            return -1;
//        }
//        else if(key == 32) // SPACE
//        {
//            cout << "Collected " << imageCount<< " images" << endl;
//            
//            ostringstream ostr; // the next three rows, convert number to string
//            ostr << imageCount;
//            string theNumberString = ostr.str();
//            
//            string filepath = "./images/IK/";
//            imwrite(filepath + theNumberString + ".jpg",frame); // writing an image to folder
//            imageCount++;
//        }
//    }
    return imageCount;
}

vector<Point2f> SingleCalibration::collectPoints(Mat image,bool camCheck)
{
    imageSize = image.size(); // image size(for instance 1356x1024) 
    cout << imageSize << endl; 
    vector<Point2f> currentPoints; // Points from the given image 
    const int maxScale = 1;  
    Mat gray;  
    convertToGray(image,gray); // convert original frame to gray
    
    if(camCheck)
    {
        bitwise_not(gray, gray);   // ! ONLY FOR IK CAMERAS ! // inverts every bit of an array
    }
    
    for(int s = 1; s <= maxScale; s++)
    {
        Mat tmpImage = gray;
        if(s > 1)
        {
            resize(gray, tmpImage, Size(0,0), s, s, CV_INTER_CUBIC);
        }       
        
        currentPoints = ptrCalibrator->collectFramePoints(tmpImage); // here we got specific corners points from the image

        if( currentPoints.size() > 0) // in case we get any points
        {
            for(uint j=0; j < currentPoints.size(); j++) //25 points 
            {
                currentPoints[j].x = currentPoints[j].x / s;
                currentPoints[j].y = currentPoints[j].y / s;
            }
            imagePoints.push_back(currentPoints);  // every points (x,y)
            break;
        }
    }   
    
    showPoints(gray,currentPoints); //Show points ont he image
    return currentPoints;
}

void SingleCalibration::showPoints(Mat image, vector<Point2f> & corners)
{
    if(corners.size() > 0)  // 25 points (x,y) > 0
    {
        Mat cornersImage;
        cvtColor(image,cornersImage, CV_GRAY2BGR); // convert To RGB again
        drawChessboardCorners(cornersImage, ptrCalibrator->getChessboardSize(),corners,true); // draw a specific points on the image 
        imshow_my("Corners", cornersImage);
    } else {
        imshow_my("Corners", image);
        cout << "no points found" << endl;
    }
    waitKey(10000);
}

void SingleCalibration::imshow_my(const String& winname, Mat& mat)
{
    Size maxSize(1600,1200);
    Mat resized;
    if(mat.cols > maxSize.width || mat.rows > maxSize.height) // resize image if it's large then 1600x1200
    {
        resize(mat, resized, maxSize);
        imshow(winname, resized);
    } else 
    {
        imshow(winname, mat);
    }
}

int SingleCalibration::calibrate(bool camCheck)
{
    cout << "The process of calibration" << endl;
    string folder = "./IK/*.jpg";
    vector<String> filename; // vector for saving all files which we found in folder
    glob(folder,filename); // the function for searching specific files
    cout << "The numbers of frames " << filename.size() << endl; 
    
    for(int i = 0; i < filename.size(); i++) 
    {
        cout << "Loading " << i+1 <<"........."  << filename[i] << endl;
        Mat image = imread(filename[i]); // read the files from the vector
        //cout << image << endl;
        collectPoints(image,camCheck); // main function for collecting point from a given image
    }
    calib();  
    return 0;
}

void SingleCalibration::calib()
{
    vector< vector<Point3f> > objectPoints; 
    int samplesCounter = imagePoints.size(); // 25 Points (x,y)
    for (int i=0; i < samplesCounter; ++i) 
    {
        objectPoints.push_back(ptrCalibrator->collectObjectPoints());  //from (x,y) to (x,y,z)  
    }
    
    Mat m,d; // m - cameraMatrix && d - distortionCoeffs
    vector <Mat> t,r; // rotaion and translation vector
     
    cout << "Image Points Size : "<< imagePoints.size() << endl; 
    double err = calibrateCamera(objectPoints, imagePoints, imageSize, m, d, r, t,
                                    CV_CALIB_FIX_K4 |
                                    CV_CALIB_FIX_K5 |
                                    CV_CALIB_FIX_K6 |
                                    CV_CALIB_FIX_K3 |
                                    CV_CALIB_ZERO_TANGENT_DIST,
                                    TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));
    cout << "Calibration error: " << err << endl; // At the end we got Pre-error of calibration
    saveCalib(m,d, imageSize);
}

void SingleCalibration::saveCalib(Mat m, Mat d, Size imageSize)
{
    const char* filename = "./IK/intrinsics.yml";
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
    const char* filename = "./IK/intrinsics.yml";
    FileStorage fs(filename, CV_STORAGE_READ);
    if (fs.isOpened()) 
    {
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
    cout << "2. Calibrate cameras. " << endl;
    cout << "3. Field of view. " << endl; 
    cout << "Choose the option: ";
    
    cin >> option;     
    SingleCalibration sc;    
    switch(option) 
    {
        case 1: 
        {
            //string source = "rtsp://192.168.0.162/live/main";         
            //string source = "rtsp://192.168.1.168/video_1";  
            //rtsp://121.23.46.168/video_1
            string source = "rtsp://192.168.1.168/video_1"; 
            //string source = "rtsp://192.168.0.111/onvif/media/PRF00.wxp"; 
            VideoCapture cap(source);         
            return sc.collectImages(cap);
            break;
        }
        case 2:
        {   
            cout << "1 - Calibration for TV camera.  " << endl;
            cout << "2 - Calibration for IK camera.  " << endl;
            int camOption = 1;
            bool camCheck = 0;
            cin >> camOption;
            
            if(camOption == 1) 
                camCheck = false;
            else if(camOption == 2) 
                camCheck = true;
            else 
                return 0;
            
            
            return sc.calibrate(camCheck);
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
