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
#define BOARD_SIZE_WIDHT    9   //4
#define BOARD_SIZE_HEIGHT   6   //11
#define SQUARE_SIZE         25  //45.5

#define ASIZE_IMAGES 10



unique_ptr<std::thread> cam0_ptr;
unique_ptr<std::thread> cam1_ptr;
atomic<bool> threadRunning(true);
atomic<bool> hasFrame0(false);
atomic<bool> hasFrame1(false);
mutex m0,m1;

TwoCamerasCalibrator::TwoCamerasCalibrator() 
{
    //_calibrator = new PointsCollectorCircles(4, 11, 44, true);
    _calibrator = new PointsCollectorChess(9,6,11);
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

void convToGray(const Mat& in, Mat& out)
{
    if(in.channels() == 3)
        cvtColor(in, out, CV_BGR2GRAY);
    else
        in.copyTo(out);
}

vector<Point2f> TwoCamerasCalibrator::collectPoints(Mat image)
{
   
    vector<Point2f> currentPoints;
    const int maxScale = 1;
    
    Mat gray;
    convToGray(image, gray);
    
    
    for(int s = 1; s <= maxScale; s++)
    {
        Mat tmpImage = gray;
        if (s > 1)
        {
            resize(gray,tmpImage,Size(0,0),s,s,CV_INTER_CUBIC);
        }      
        //cout << tmpImage << endl;
        currentPoints = _calibrator->collectFramePoints(tmpImage);
          
        if(currentPoints.size() > 0)
        {
            for(uint j = 0; j < currentPoints.size(); j++)
            {
                currentPoints[j].x /= s;
                currentPoints[j].y /= s;
            }
            break;
        }
    }
    showPoints(gray, currentPoints);   
    return currentPoints;
}




void TwoCamerasCalibrator::showPoints(Mat image, vector<Point2f>& corners)
{
    if(corners.size() > 0)
    {
        Mat cornersImage;
        cvtColor(image,cornersImage, CV_GRAY2BGR);
        drawChessboardCorners(cornersImage, _calibrator->getChessboardSize(),corners, true);
        imshow("corners", cornersImage);
        waitKey(100);
    }
}


int TwoCamerasCalibrator::stereoCalibrateRectificate(const vector<string>& imagelist, Size boardSize, float squareSize)
{
    //cout << imagelist.size() << endl;    
    
    if(imagelist.size() % 2 != 0)
    {
        cout << "Error: the image list contains odd (non-even) number of element";
        return -1;
    }
       
    const int maxScale = 2;  
    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    
    Size imageSize;   
    int i , j , k;
    int nimages = (int)imagelist.size() / 2; // 30 frames    
    
    //cout << nimages << endl;
    
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    
    //cout << imagePoints[0].size() << " x "<<imagePoints[1].size() << endl;
    
    vector<string> goodImageList;
  
    for (i = j = 0; i < nimages; i++)
    {
        for(k = 0; k < 2; k++)
        {
            //cout << i * 2 + k << endl;
            const string& filename = imagelist[i * 2 + k];
            
            //cout << filename << endl;
            
            Mat img = imread(filename, 0);
            
            //cout << img << endl;
            
            if(img.empty())
                break;           
            
            if(imageSize == Size())
            {
                imageSize = img.size();
            }
            
            //cout << imageSize << endl;
            
            Mat dst;
            //bitwise_not(img, dst);
            collectPoints(img);
            imagePoints[k][j] = collectPoints(img);
            //cout << imagePoints[k][j] << endl;
            
        }   
        //cout << k << endl;
        if(k == 2)
        {
            goodImageList.push_back(imagelist[i * 2]);
            goodImageList.push_back(imagelist[i * 2 + 1]);
            objectPoints.push_back(_calibrator->collectObjectPoints());
            j++;
        }
    }
    cout << j << " pairs have been successfully detected .\n";
    nimages = j;
    
    if(nimages < 2)
    {
        cout << "Error: too little pairs to run the calibration\n";
        return -1;
    }
    
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);
    
    cout << "Stereo calibration is loading and running...\n";
    Mat cameraMatrix[2];
    Mat distCoeffs[2];
    Mat R, T, E, F;
    vector<Mat> r, t;
    cout << __FILE__ << " " << __LINE__ << endl;
//    for(int x = 0; x < j; x++)
//    {
//        cout << objectPoints[x] << endl;
//        cout << imagePoints[0][x] << endl;
//        cout << imagePoints[1][x] << endl;
//        cout << "-----------------------------------------" << endl;
//    }
    cout << "objectPoints.size() " << objectPoints.size() << endl;
    cout << "imagePoints[0].size() " << imagePoints[0].size() << endl;
    cout << "imagePoints[1].size() " << imagePoints[1].size() << endl;
    cout << "imageSize " << imageSize << endl;

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                cameraMatrix[0], distCoeffs[0],
                                cameraMatrix[1], distCoeffs[1],
                                imageSize, R, T, E, F,
                                CV_CALIB_FIX_ASPECT_RATIO,
                                TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));
    cout << "Stereo Calibrate done with RMS error = " << rms << endl;
    
    
    
    // ---------------------- (BEGIN) CALIBRATION QUALITY CHECK ------------------------------------
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for (i = 0; i < nimages; i++) 
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for (k = 0; k < 2; k++) 
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
        }
        for (j = 0; j < npt; j++) 
        {
            double errij = fabs(imagePoints[0][i][j].x * lines[1][j][0] +
                                imagePoints[0][i][j].y * lines[1][j][1] + 
                                lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x * lines[0][j][0] +
                                imagePoints[1][i][j].y * lines[0][j][1] + 
                                lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " << err / npoints << endl;
    // ---------------------- (END) CALIBRATION QUALITY CHECK ------------------------------------


}




int TwoCamerasCalibrator::twoCamerasCalibration()
{
    int option = 1 ; // collect images by default       
    cout << "1. Collect images from both cameras(TV,IK) for StereoCalibration!" << endl;
    cout << "2. Stereo Calibration" << endl;
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
            for(int i = 1; i <= (NUM_IMAGES * 2); ++i)
            {
                string name = "/home/ilya/NetBeansProjects/CameraMerging/DoubleImages/cam_" + to_string(i) + ".jpg";
                //cout << name << endl;
                imagelist.push_back(name);
            }    
            return stereoCalibrateRectificate(imagelist, boardSize, SQUARE_SIZE);
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