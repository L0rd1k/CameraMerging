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
#define BOARD_SIZE_WIDHT    4   //4 9
#define BOARD_SIZE_HEIGHT   11   //11 6 
#define SQUARE_SIZE         39  //45.5 25
#define ASIZE_IMAGES 10

struct CameraContext {
    std::string url;
    unique_ptr<std::thread> cameraThread;
    bool threadRunning = true;
    bool hasFrame = false;
    std::mutex m;

    Mat frameCamDistorted;
    Mat intrinsicsMatrix, distortions;
    Mat intrinsicsMatrixUndistort, distortionsUndistort;
    Mat frameCam;
    vector<string> imagelist;
    
    Size boardSize;
    bool inverting = false;    
    Mat cornersImage;
    
    int flags;
    vector<Point2f> pt;
    bool circlesResult;
};

CameraContext cameras[2];

TwoCamerasCalibrator::TwoCamerasCalibrator() {
    _calibrator = new PointsCollectorCircles(4, 11, 39, true);
    //_calibrator = new PointsCollectorChess(9,6,11);
}

TwoCamerasCalibrator::~TwoCamerasCalibrator() {

}


int loadMatrix(string filename, string name, Mat &m) {
    FileStorage fs;
    int result = fs.open(filename, CV_STORAGE_READ); // checking for opening file
    if (result) {
        fs[name] >> m;
        fs.release();
        if (m.rows > 0) {
            return EXIT_SUCCESS; // return EXIT_SUCCESS;
        }
        else {
            cerr << "Matrix " << name << " was empty in file " << filename << endl;
        }
    }
    else {
        cerr << "Matrix " << name << " not found in file" << filename << endl;
    }
    return EXIT_FAILURE; // return EXIT_FAILURE;
}

int TwoCamerasCalibrator::loadCalibrations(int option) 
{  
    if (loadMatrix("SingleCalibration/TV-NetworkCamera/intrinsics.yml", "m", cameras[0].intrinsicsMatrix) == EXIT_FAILURE) {
        //"SingleCalibration/TV-NetworkCamera/intrinsics.yml"
        return 1;
    }  
    if (loadMatrix("SingleCalibration/TV-NetworkCamera/intrinsics.yml", "d", cameras[0].distortions) == EXIT_FAILURE) {
        return 1;
    }
    if (loadMatrix("SingleCalibration/TV/intrinsics.yml", "m", cameras[1].intrinsicsMatrix) == EXIT_FAILURE) {
        return 1;
    }
    if (loadMatrix("SingleCalibration/TV/intrinsics.yml", "d", cameras[1].distortions) == EXIT_FAILURE) {
        return 1;
    }
    
    if (option == 3)
    {
        if (loadMatrix("DoubleCalibration/DoubleImages/extrisics.yml", "R", R) == EXIT_FAILURE) {
            return 1;
        }
        if (loadMatrix("DoubleCalibration/DoubleImages/instrisics.yml", "M0", cameras[0].intrinsicsMatrixUndistort) == EXIT_FAILURE) {
            return 1;
        } 
        if (loadMatrix("DoubleCalibration/DoubleImages/instrisics.yml", "D0", cameras[0].distortionsUndistort) == EXIT_FAILURE) {
            return 1;
        }
        if (loadMatrix("DoubleCalibration/DoubleImages/instrisics.yml", "M1", cameras[1].intrinsicsMatrixUndistort) == EXIT_FAILURE) {
            return 1;
        }
        if (loadMatrix("DoubleCalibration/DoubleImages/instrisics.yml", "D1", cameras[1].distortionsUndistort) == EXIT_FAILURE) {
            return 1;
        } 
    }
    
    return EXIT_SUCCESS; // return EXIT_SUCCESS;
}

int TwoCamerasCalibrator::grabPicture(int id) {
    
    auto& ctx = cameras[id]; 
    
    cout << "Start grabbing " << ctx.url << endl;
    
    VideoCapture cap(ctx.url);
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH, MAX_CAMERA_RESOLUTION);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, MAX_CAMERA_RESOLUTION);
    cap.set(CV_CAP_PROP_FPS, FPS);

    if (!cap.isOpened()) {
        cout << "Error opening url" << ctx.url << endl;
        return EXIT_FAILURE;
    }
    
    while (ctx.threadRunning) {        
        cap.grab();
        cap.retrieve(ctx.frameCamDistorted);
        if (ctx.frameCamDistorted.rows > 0){
            lock_guard<std::mutex> guard(ctx.m);
            if(id == 0) {
                bitwise_not(ctx.frameCamDistorted, ctx.frameCam);                
            } else {
                ctx.frameCam = ctx.frameCamDistorted.clone();
            }
            ctx.hasFrame = true;
        }       
    }
    
    return EXIT_SUCCESS;
}

void TwoCamerasCalibrator::startCameras() {
    //cameras[0].url = "rtsp://192.168.0.111/onvif/media/PRF00.wxp";
    cameras[0].url = "rtsp://192.168.0.162/live/main";
    cameras[1].url = "rtsp://192.168.1.168/video_1";
    
    cameras[0].cameraThread = unique_ptr<std::thread>(new thread(&TwoCamerasCalibrator::grabPicture, this, 0));
    cameras[1].cameraThread = unique_ptr<std::thread>(new thread(&TwoCamerasCalibrator::grabPicture, this, 1));
    
    cameras[0].inverting = true;
    cameras[1].inverting = false;
}

void TwoCamerasCalibrator::stopCameras() {
    cameras[0].threadRunning = false;
    cameras[1].threadRunning = false;
    cameras[0].cameraThread->join(); // wait until second camera ended its job
    cameras[1].cameraThread->join(); // wait until first camera ended its job
}

int TwoCamerasCalibrator::collectImages() {
    mkdir("./DoubleCalibration/DoubleImages", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cout << "Press 'x' to capture a new image" << endl;
    
    int key = 0;
    int count = 1;
    
    while (key != 27) {
        for(int id = 0; id < 2; id++){
            auto& ctx = cameras[id];
            if(ctx.hasFrame) {
                std::lock_guard<std::mutex> guard(ctx.m);
                cvtColor(ctx.frameCam,ctx.frameCam,CV_BGR2GRAY);
                ctx.flags = CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING;
                ctx.circlesResult = findCirclesGrid(ctx.frameCam, _calibrator->getChessboardSize(),ctx.pt,ctx.flags);
                cvtColor(ctx.frameCam,ctx.cornersImage, CV_GRAY2BGR);
            
                if(ctx.circlesResult){
                    drawChessboardCorners(ctx.cornersImage,_calibrator->getChessboardSize(),ctx.pt,true);
                    imshow("Corners" + to_string(id),ctx.cornersImage);
                    ctx.pt.clear();
                    if(key == 'x' || key == 120) {
                        for (int id = 0; id < 2; id++) {
                            auto &ctx = cameras[id];
                            string name = "./DoubleCalibration/DoubleImages/cam_" + to_string(count++) + ".jpg";
                            cout << count-1 << " - " << name << endl;
                            ctx.imagelist.push_back(name);
                            std::lock_guard<std::mutex> guard1(ctx.m);
                            imwrite(name,ctx.frameCam);
                            ctx.frameCam = false;
                        }
                    }  
                }
                else {
                    imshow("Corners" + to_string(id),ctx.frameCam);
                }
                ctx.hasFrame = false;
            }
        }
        key = waitKey(1);
    }
    return EXIT_SUCCESS; // EXIT_SUCCESS
}

void convToGray(const Mat& in, Mat& out) {
    if (in.channels() == 3)
        cvtColor(in, out, CV_BGR2GRAY);
    else
        in.copyTo(out);
}

Points2f TwoCamerasCalibrator::collectPoints(Mat image) {
    Points2f currentPoints;
    Mat gray;
    convToGray(image, gray);
    currentPoints = _calibrator->collectFramePoints(gray);
    showPoints(gray, currentPoints);
    return currentPoints;
}

void TwoCamerasCalibrator::showPoints(Mat image, Points2f& corners) {
    if (corners.size() > 0) {
        Mat cornersImage;
        cvtColor(image, cornersImage, CV_GRAY2BGR);
        drawChessboardCorners(cornersImage, _calibrator->getChessboardSize(), corners, true);
        imshow("Corners", cornersImage);
    } else
    {
        imshow("Corners", image);    
    }
    waitKey(10000);
}

int TwoCamerasCalibrator::getPoints(vector<Points2f> *imagePoints, vector<Points3f>& objectPoints) {
    
    cout << imagelist.size() << endl;
    
    if (imagelist.size() == 0 || imagelist.size() % 2 != 0) 
    {
        cout << "Error: the image list contains odd (non-even) number of element";
        return -1;
    }   
    
    int nimages = (int)imagelist.size();   
    int nmb = 0;
//    imagePoints[0].resize(nimages);  
//    imagePoints[1].resize(nimages);
    for(int i = 0; i < nimages; i++) {  
        const string& filename = imagelist[i];
        Mat img = imread(filename, 0);
        if (img.empty()) {
            cout << "Image is empty!" << endl;
            break;
        }
        int id = i % 2; // check the id of camera 
        cout << i+1 << ") " << id << " - " << img.size() << " - " << filename << endl;
        imagePoints[id].push_back(collectPoints(img));
        if(id == 0) {
            objectPoints.push_back(_calibrator->collectObjectPoints());
            nmb++;
        } 
    }
    return nmb; 
}

// Checks if a matrix is a valid rotatiosn matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());  
    return  norm(I, shouldBeIdentity) < 1e-6;   
}
// Convert a Rotation Matrix to Euler Angles.
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    
    assert(isRotationMatrix(R));    
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2)) * (180/3.14);
        y = atan2(-R.at<double>(2,0), sy) * (180/3.14);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0)) * (180/3.14);
    }
    else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1)) * (180/3.14);
        y = atan2(-R.at<double>(2,0), sy) * (180/3.14);
        z = 0;
    }
    return Vec3f(x, y, z);   
}

int TwoCamerasCalibrator::stereoCalibrateRectificate(Size boardSize, float squareSize) {
    
    vector<Points2f> imagePoints[2]; // two cameras
    vector<Points3f> objectPoints;
    
    Mat cameraMatrix[2];
    Mat distCoeffs[2];
    
    int nimages = getPoints(imagePoints, objectPoints); // Get the number of images pairs where pattern was detected
    if(nimages == 0) {
        cout << "No images was detected" << endl;
        return -1;
    }
    
    cout << "There were " << nimages << " pairs of image detected." << endl;
    
    Mat img = imread(imagelist[0], 0); // ????????? WHY FIRST CAMERA
    
    Size imageSize = img.size(); // size of the first camera
    
    cout << "...Stereo calibration is loading and running..." << endl;
    
    imagePoints[0].resize(nimages); // 22 frames
    imagePoints[1].resize(nimages); // 22 frames
      
    //Get matrix from SingleOne Calibration
    cameraMatrix[0] = cameras[0].intrinsicsMatrix;
    cameraMatrix[1] = cameras[1].intrinsicsMatrix;
    
    //Get ditortion from Single Calibration
    distCoeffs[0] = cameras[0].distortions;
    distCoeffs[1] = cameras[1].distortions;
    
    Mat R, T, E, F;
    
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
            cameraMatrix[0], distCoeffs[0],
            cameraMatrix[1], distCoeffs[1],
            Size(), R, T, E, F, 
            CV_CALIB_FIX_INTRINSIC | CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_ASPECT_RATIO,
            TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));
    
    cout << "Stereo Calibrate done with RMS error = " << rms << endl;
 
    // ---------------------- (BEGIN) CALIBRATION QUALITY CHECK ------------------------------------
    
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    
    double err = 0;
    int nPoints = 0;
    
    cout << "ImagePoints" << imagePoints[0].size() << endl;
    
    vector<Vec3f> lines[2];
    
    int i, j, k; // iterators
    
    for (i = 0; i < nimages; i++) { // 22 go through pairs of camera's image
        int numberPoints = (int) imagePoints[0][i].size(); // 44 
        Mat imgPoints[2];
        
        for (k = 0; k < 2; k++) {   // for each of camera's make an undistort && computeCorrespondEpilines  
            imgPoints[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgPoints[k], imgPoints[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]); // Observed point coord. -> ideal points coord.           
            computeCorrespondEpilines(imgPoints[k], k + 1, F, lines[k]); //
        }     
        
        for (j = 0; j < numberPoints; j++) {    
            double errij = fabs(imagePoints[0][i][j].x * lines[1][j][0] + imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x * lines[0][j][0] + imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
            err = err + errij;
        }
        nPoints = nPoints + numberPoints; 
    }
    cout << "Average epipolar error = " << err / nPoints << endl;
    // ---------------------- (END) CALIBRATION QUALITY CHECK --------------------------------------
    
    // ---------------------- (BEGIN) Save Intrinsic parameters (Matrix and Distortion coeff) ------------------------------------  
    FileStorage fs("./DoubleCalibration/DoubleImages/instrisics.yml", FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "M0" << cameraMatrix[0] << "D0" << distCoeffs[0] <<
              "M1" << cameraMatrix[1] << "D1" << distCoeffs[1];
        fs.release();
    }
    else {
        cout << "Error: can not save the intrinsic parameters\n";
    }
    // ---------------------- (END) Save Intrinsic parameters ------------------------------------ 
                                                                                                    
    // ---------------------- (BEGIN) Save Extrinsic parameters ------------------------------------ 
    //The next step is to find a feature's position in space from the 2 images I get.
    
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    
    // Computes rectification transforms for each head of a calibrated stereo camera.    
    stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
                imageSize, //Size of the image used for stereo calibration.
                R, //Rotation matrix between the coordinate systems of the first and the second cameras.
                T, //Translation vector between coordinate systems of the cameras.
                R1, //Output 3x3 rectification transform (rotation matrix) for the first camera.
                R2, //Output 3x3 rectification transform (rotation matrix) for the second camera.
                P1, //Output 3x4 projection matrix in the new (rectified) coordinate systems for the first camera.
                P2, //Output 3x4 projection matrix in the new (rectified) coordinate systems for the second camera.
                Q,  //Output 4x4 disparity-to-depth mapping matrix
                CALIB_ZERO_DISPARITY, 1, imageSize,
                &validRoi[0], &validRoi[1]);
    
    fs.open("./DoubleCalibration/DoubleImages/extrisics.yml", FileStorage::WRITE);
    
    cout << rotationMatrixToEulerAngles(R) << endl; // get the angle from rotation matrix 
    
    if (fs.isOpened()) {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 <<
              "P1" << P1 << "P2" << P2 << "Q" << Q << "F" << F;
    } else {
        cout << "Error: Extrinsic parameters can't be save" << endl;
    }
    // ---------------------- (END) Save Extrinsic parameters ------------------------------------ 
    
    // ---------------------- (BEGIN) Computes the Undistortion and Rectification transformation map ------------------------------------ 
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
    Mat rmap[2][2];
    
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int width, height;
    
    // Create and calculate the size of canvas for rectification
    if (!isVerticalStereo) {
        sf = 600. / MAX(imageSize.width, imageSize.height); // 600/768
        width = cvRound(imageSize.width * sf);
        height = cvRound(imageSize.height * sf);
        canvas.create(height, width * 2, CV_8UC3);
    } 
    else {
        sf = 300. / MAX(imageSize.width, imageSize.height); // 300/768
        width = cvRound(imageSize.width * sf);
        height = cvRound(imageSize.height * sf);
        canvas.create(height * 2, width, CV_8UC3);
    }
    
    Mat buffer;
    for (i = 0; i < nimages; i++) {
        for (k = 0; k < 2; k++) {
            Mat remapImg, cimg;
            Mat img = imread(imagelist[i * 2 + k], 0); // original TV frames  
            remap(img, remapImg, rmap[k][0], rmap[k][1], INTER_LINEAR); //

            cvtColor(remapImg, cimg, COLOR_GRAY2BGR); // convert to RGB         
            
            Mat canvasPart;
            
            if(!isVerticalStereo) {
                canvasPart = canvas(Rect(width*k, 0, width, height));
            } else {
                canvasPart = canvas(Rect(0, height*k, width, height));
            }
 
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
 
            Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y * sf),
                    cvRound(validRoi[k].width * sf), cvRound(validRoi[k].height * sf));
            
            rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);            
            //imshow("CanvasPart",canvasPart);
            buffer = remapImg;
        }
        
        
        if (!isVerticalStereo) {
            for (j = 0; j < canvas.rows; j += 16) {
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
            }
        } else {
            for (j = 0; j < canvas.cols; j += 16) {
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
            }
        }
        
        imshow("Rectified", canvas);
        
        char c = (char) waitKey();
        if (c == 27 || c == 'q' || c == 'Q')
            break;
    }
    
    // ---------------------- (END) Computes the undistortion and rectification transformation map ------------------------------------ 
   
    return EXIT_SUCCESS;
}
int TwoCamerasCalibrator::mergeImages() {
    
    Mat frameCamRotated;
    int key = 0;
    
    while (key != 27) {     
        for( int id = 0; id < 2; id++)
        {
            auto& ctx = cameras[id];
            if(ctx.hasFrame)
            {
                std::string winName = "FrameCam " + to_string(id);
                cout <<"Size: id-" + to_string(id) << " "<< ctx.frameCam.size() << endl;
                imshow(winName, ctx.frameCam);
                ctx.hasFrame = false;
            }            
        }        

        key = waitKey(10);        
        if(cameras[1].frameCam.size().height > 0 && cameras[0].frameCam.size().height > 0)
        {
            DualCameraAligner dca(cameras[0].intrinsicsMatrix, cameras[1].intrinsicsMatrix, R);
            //frameCamRotated = dca.align(cameras[0].frameCam); // align IK camera
            frameCamRotated = dca.align(cameras[1].frameCam); // align TV camera
            //imshow("FCR",frameCamRotated);
            DualCameraMerger dcm;
            //Mat result = dcm.merge(frameCamRotated, cameras[1].frameCam);
            Mat result = dcm.merge(frameCamRotated,cameras[0].frameCam);
            if(result.cols > 0)
            {
                imshow("Result", result);
            }
        }
    }
    return EXIT_SUCCESS;
}

int TwoCamerasCalibrator::twoCamerasCalibration() {
    int option = 1; // collect images by default       
    cout << "1. Collect images from both cameras(TV,IK) for StereoCalibration!" << endl;
    cout << "2. Stereo Calibration" << endl;
    cout << "3. Frames merging" << endl;
    cout << "Choose the option: ";
    cin >> option;
    
    if (loadCalibrations(option) == EXIT_FAILURE) 
    {
        return EXIT_FAILURE;
    }
        
    switch (option) {
        case 1:{
            startCameras();
            collectImages();
            stopCameras();
            break;
        }
        case 2: {
            
            Size boardSize;
            boardSize.width = BOARD_SIZE_WIDHT;
            boardSize.height = BOARD_SIZE_HEIGHT;
//            for (int i = 1; i <= (NUM_IMAGES * 2); ++i) {
//                string name = "./DoubleCalibration/DoubleImages/cam_" + to_string(i) + ".jpg";
//                imagelist.push_back(name);
//            }   
            string folder = "./DoubleCalibration/DoubleImages/*.jpg";
            vector<String> filename;
            glob(folder,filename);
            
            sort(filename.begin(),filename.end(), [](const string& s1, const string& s2){
                if (s1.length() < s2.length())
                    return true;
                if (s2.length() < s1.length())
                    return false;
                else
                    return (s1 < s2);
            });
            
            for(int i = 0; i < filename.size(); i++) {
                //int vl = stoi(filename[i]);    
                cout /*<< vl*/ << " == " << filename[i] << endl;
            }   
            cout << "The numbers of frames " << filename.size() << endl;
            for(int i = 0; i < filename.size(); i++) {
                imagelist.push_back(filename[i]);
            }
            return stereoCalibrateRectificate(boardSize, SQUARE_SIZE);
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
            return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}