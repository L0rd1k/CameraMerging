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
std::mutex m0,m1;

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
        } 
        else
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

int TwoCamerasCalibrator::grabPicture(int camValue, VideoCapture cap, int camera) 
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
        cap.retrieve(frameCamDistorted[camValue]);
        if(frameCamDistorted[camValue].size().height > 0)
        {
            if(camValue == 0)
            {
                lock_guard<std::mutex> guard(m0);
                hasFrame0 = true;
            }
            else
            {
                lock_guard<std::mutex> guard(m1);
                hasFrame1 = true; 
            }
            
            frameCam[camValue] = frameCamDistorted[camValue]; 
        }
        cout << frameCam[camValue].size() << endl;        
//        if(camera == 0)
//        {
//            cap.retrieve(frameCamDistorted[0]);
//            if(frameCamDistorted[0].size().height > 0)
//            {
//                lock_guard<std::mutex> guard(m0);
//                frameCam[0] = frameCamDistorted[0];
//                hasFrame0 = true;
//            }
//            cout << frameCam[0].size() << endl;
//        }
//        else
//        {
//            cap.retrieve(frameCamDistorted[1]);
//            if(frameCamDistorted[1].size().height > 0)
//            {
//                lock_guard<std::mutex> guard(m1);
//                frameCam[1] = frameCamDistorted[1];
//                hasFrame1 = true;
//            }
//            cout << frameCam[1].size() << endl;
//        }
    }
    return EXIT_SUCCESS;
}


void TwoCamerasCalibrator::startCameras()
{
    // FIRST CAMERA
    VideoCapture cap0("rtsp://192.168.0.162/live/main");
    //rtsp://121.23.46.111:554/onvif/media/PRF08.wxp
    //rtsp://121.23.46.168/video_1
    cap0.set(CV_CAP_PROP_FRAME_WIDTH, MAX_CAMERA_RESOLUTION);
    cap0.set(CV_CAP_PROP_FRAME_HEIGHT, MAX_CAMERA_RESOLUTION);
    resolutionCamX[0] = static_cast<int>(cap0.get(CV_CAP_PROP_FRAME_WIDTH));
    resolutionCamY[0] = static_cast<int>(cap0.get(CV_CAP_PROP_FRAME_HEIGHT));
    cap0.set(CV_CAP_PROP_FPS, FPS);
       
    // SECOND CAMERA
    //VideoCapture cap1("rtsp://192.168.1.168/video_1");
    //rtsp://121.23.46.111:554/onvif/media/PRF00.wxp
    //rtsp://192.168.0.162/live/main
    VideoCapture cap1("rtsp://121.23.46.168/video_1");
    cap1.set(CV_CAP_PROP_FRAME_WIDTH, MAX_CAMERA_RESOLUTION);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT, MAX_CAMERA_RESOLUTION);
    resolutionCamX[1] = static_cast<int>(cap1.get(CV_CAP_PROP_FRAME_WIDTH));
    resolutionCamY[1] = static_cast<int>(cap1.get(CV_CAP_PROP_FRAME_HEIGHT));
    cap1.set(CV_CAP_PROP_FPS, FPS);
       
    cam0_ptr = unique_ptr<std::thread>(new thread(&TwoCamerasCalibrator::grabPicture, this, 0, cap0, 0));
    cam1_ptr = unique_ptr<std::thread>(new thread(&TwoCamerasCalibrator::grabPicture, this, 1, cap1, 1));
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
        
        
        if(hasFrame0) // TV
        {
           
            frameCamCopy[0] = frameCam[0].clone();   
            imshow("Frame Camera [0]", frameCamCopy[0]);
            hasFrame0 = false;
        }
        
        if(hasFrame1) //IK
        {
            frameCamCopy[1] = frameCam[1].clone();
            imshow("Frame Camera [1]", frameCamCopy[1]);
            hasFrame1 = false;
        }
        

        if(key == 'x')
        {
            string name1 = "/home/ilya/NetBeansProjects/CameraMerging/DoubleImages/cam_" + to_string(count++) + ".jpg";
            imagelist.push_back(name1);
            string name2 = "/home/ilya/NetBeansProjects/CameraMerging/DoubleImages/cam_" + to_string(count++) + ".jpg";
            imagelist.push_back(name2);
            
            imwrite(name1, frameCamCopy[0]);
            imwrite(name2, frameCamCopy[1]);
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
    cout << j << " pairs have been successfully detected.\n";
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
    cout << "ObjectPoints.size() " << objectPoints.size() << endl;
    cout << "ImagePoints[0].size() " << imagePoints[0].size() << endl;
    cout << "ImagePoints[1].size() " << imagePoints[1].size() << endl;
    cout << "ImageSize " << imageSize << endl;

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
    int n_points = 0;
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
        
        n_points += npt;
    }
    
    cout << "Average epipolar error = " << err / n_points << endl;
    // ---------------------- (END) CALIBRATION QUALITY CHECK ------------------------------------

    // ---------------------- (BEGIN) Savev Intrinsic parameters ------------------------------------
    
    FileStorage fs("/home/ilya/NetBeansProjects/CameraMerging/DoubleImages/instrisics.yml", FileStorage::WRITE);
    if(fs.isOpened())
    {
        fs << "M0" << cameraMatrix[0] << "D0" << distCoeffs[0] <<
              "M1" << cameraMatrix[1] << "D1" << distCoeffs[1];  
        fs.release();
    } 
    else
    {
        cout << "Error: can not save the intrinsic parameters\n";
    }
    
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, 
                  &validRoi[0], &validRoi[1]) ;
    fs.open("/home/ilya/NetBeansProjects/CameraMerging/DoubleImages/instrisics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << 
              "P1" << P1 << "P2" << P2 << "Q" << Q << "F" << F;
    }
    else
    {
        cout << "Error: Extrinsic parameters can't be save" << endl;
    }
    
    bool isVerticalStereo = fabs(P2.at<double>(1,3)) > fabs(P2.at<double>(0,3));
    
    Mat rmap[2][2];
    initUndistortRectifyMap(cameraMatrix[0],distCoeffs[0],R1,P1,imageSize,CV_16SC2,rmap[0][0],rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if(!isVerticalStereo)
    {
        sf = 600. / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h, w * 2, CV_8UC3);
    }
    else
    {
        sf = 300. / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h * 2, w, CV_8UC3);
    }
    
    Mat buffer;
    for(i = 0; i < nimages; i++)
    {
        for(k = 0; k < 2; k++)
        {
            Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            
            Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y *sf),
                      cvRound(validRoi[k].width * sf), cvRound(validRoi[k].height * sf));
            rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
            buffer = rimg;
        }
        //imshow("rimg", buffer);
        if(!isVerticalStereo)
        {
            for(j = 0; j < canvas.rows; j += 16)
            {
                line(canvas, Point(0,j), Point(canvas.cols,j), Scalar(0, 255, 0), 1, 8);    
            }
        }
        else
        {
            for(j = 0; j < canvas.cols; j += 16)
            {
                line(canvas, Point(j,0), Point(j,canvas.rows), Scalar(0, 255, 0), 1, 8);                  
            }
        }
        imshow("Rectified", canvas);
        char c = (char)waitKey();
        if (c == 27 || c == 'q' || c == 'Q')
            break;
    }
    return EXIT_SUCCESS;
}

int TwoCamerasCalibrator::mergeImages()
{
    Mat frameCamCopy[2];
    Mat frameCam0Rotated;
    int key = 0;
    while(key != 27)
    {
        if(hasFrame0)
        {
//            lock_guard<std::mutex> guard0(m0);
//            frameCamCopy[0] = frameCam[0];
            imshow("FrameCam[0]", frameCamCopy[0]);
            hasFrame0 = false;
        }
        
        if(hasFrame1) 
        {
//            lock_guard<std::mutex> guard1(m1);
//            frameCamCopy[1] = frameCam[1];
            imshow("FrameCam[1]", frameCamCopy[1]);
            hasFrame1 = false;
        }
        key = waitKey(10);
        if(frameCamCopy[1].size().height > 0 && frameCamCopy[0].size().height)
        {
            DualCameraAligner dca(intrinsicsMatrixUndistort[0], intrinsicsMatrixUndistort[1], R);
            frameCam0Rotated = dca.align(frameCamCopy[0]);
            DualCameraMerger dcm;
            Mat result = dcm.merge(frameCam0Rotated, frameCamCopy[1]);
            if(result.cols > 0)
            {
                imshow("Result",result);
            }
        }
    }
    return EXIT_SUCCESS;
}

int TwoCamerasCalibrator::twoCamerasCalibration()
{
    int option = 1 ; // collect images by default       
    cout << "1. Collect images from both cameras(TV,IK) for StereoCalibration!" << endl;
    cout << "2. Stereo Calibration" << endl;
    cout << "3. Frames merging" << endl; 
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
            startCameras();
            mergeImages();
            stopCameras();
            break;
        }
        default:
            cout << "Wrong option!!!" << endl;
            return 1; // return EXIT_FAILURE
            
    }
    return 0; // EXIT_SUCCESS
}