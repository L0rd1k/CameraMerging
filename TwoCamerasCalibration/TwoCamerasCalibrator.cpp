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

#define BOARD_SIZE_WIDHT    4   //4 9
#define BOARD_SIZE_HEIGHT   11   //11 6 
#define SQUARE_SIZE         39  //45.5 25

struct CameraContext {
    std::string url;
    unique_ptr<std::thread> cameraThread;
    bool threadRunning = true;
    bool hasFrame = false;
    std::mutex m;
    Mat frameCamDistorted;
    Mat frameCamUndistorted;
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
    //_calibrator = new PointsCollectorCircles(4, 11, 39, true);
    _calibrator = new PointsCollectorChess(9, 6, 25);
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
        } else {
            cerr << "Matrix " << name << " was empty in file!" << filename << endl;
        }
    } else {
        cerr << "Matrix " << name << " not found in file!" << filename << endl;
    }
    return EXIT_FAILURE; // return EXIT_FAILURE;
}

int TwoCamerasCalibrator::loadCalibrations(int option) {
    if (loadMatrix("SingleCalibration/new2-174/intrinsics.yml", "m", cameras[0].intrinsicsMatrix) == EXIT_FAILURE) {
        //"SingleCalibration/TV-NetworkCamera/intrinsics.yml"
        return 1;
    }
    if (loadMatrix("SingleCalibration/new2-174/intrinsics.yml", "d", cameras[0].distortions) == EXIT_FAILURE) {
        return 1;
    }
    if (loadMatrix("SingleCalibration/new2-168/intrinsics.yml", "m", cameras[1].intrinsicsMatrix) == EXIT_FAILURE) {
        return 1;
    }
    if (loadMatrix("SingleCalibration/new2-168/intrinsics.yml", "d", cameras[1].distortions) == EXIT_FAILURE) {
        return 1;
    }

    if (option == 3) {
        if (loadMatrix(workingFold + "/extrisics.yml", "R", R) == EXIT_FAILURE) {
            return 1;
        }
        if (loadMatrix(workingFold + "/instrisics.yml", "M0", cameras[0].intrinsicsMatrixUndistort) == EXIT_FAILURE) {
            return 1;
        }
        if (loadMatrix(workingFold + "/instrisics.yml", "D0", cameras[0].distortionsUndistort) == EXIT_FAILURE) {
            return 1;
        }
        if (loadMatrix(workingFold + "/instrisics.yml", "M1", cameras[1].intrinsicsMatrixUndistort) == EXIT_FAILURE) {
            return 1;
        }
        if (loadMatrix(workingFold + "/instrisics.yml", "D1", cameras[1].distortionsUndistort) == EXIT_FAILURE) {
            return 1;
        }
    }

    return EXIT_SUCCESS; // return EXIT_SUCCESS;
}

int TwoCamerasCalibrator::grabPicture(int id) {
    auto& ctx = cameras[id];
    cout << "Start grabbing...." << ctx.url << endl;
    VideoCapture cap(ctx.url);
    if (!cap.isOpened()) {
        cout << "ERROR OPENING URL!!! : " << ctx.url << endl;
        return EXIT_FAILURE;
    }
    while (ctx.threadRunning) {
        cap.grab(); // Decodes and returns the grabbed video frame
        cap.retrieve(ctx.frameCamDistorted); // Stream operator to read the next video frame.
        if (ctx.frameCamDistorted.rows > 0) { // if frame exist
            lock_guard<std::mutex> guard(ctx.m);
            /* FIXME Check this code of you use TV camera*/
            if (ctx.inverting) // case first camera
                bitwise_not(ctx.frameCamDistorted, ctx.frameCam);
            else
                ctx.frameCam = ctx.frameCamDistorted.clone();
            ctx.frameCam = ctx.frameCamDistorted.clone();
            ctx.hasFrame = true;
        }
    }
    return EXIT_SUCCESS;
}

void TwoCamerasCalibrator::startCameras() {
    //cameras[1].url = "rtsp://192.168.0.162/live/main";
    cameras[1].url = "rtsp://192.168.1.174/video_1";
    cameras[0].url = "rtsp://192.168.1.168/video_1";
    for (auto id = 0; id < 2; id++) {
        cameras[id].cameraThread = unique_ptr<std::thread>(new thread(&TwoCamerasCalibrator::grabPicture, this, id));
        cameras[id].inverting = (id % 2 == 0) ? true : false;
    }
}

void TwoCamerasCalibrator::stopCameras() {
    for (int id = 0; id < 2; id++) {
        cameras[id].threadRunning = false;
        cameras[id].cameraThread->join();
    }
}

int TwoCamerasCalibrator::testHomography() {
    cout << pnts[0] << " --- " << pnts[1] << endl;
    Mat img1_warp;
    if (!pnts[0].empty() && !pnts[1].empty()) {
        Mat H = findHomography(pnts[0], pnts[1]);
        //cout << "H:\n" << H << endl;
        warpPerspective(cameras[0].frameCam, img1_warp, H, cameras[0].frameCam.size());
        imshow("TEST", img1_warp);
    }
    cout << cameras[0].frameCam.size().height << " X " << cameras[1].frameCam.size().height << endl;
    if (cameras[1].frameCam.size().height > 0 && cameras[0].frameCam.size().height > 0) {
        DualCameraMerger dcm;
        Mat result = dcm.merge(img1_warp, cameras[1].frameCam);
        if (result.cols > 0) {
            imshow("Result", result);
        }
    }
}

int TwoCamerasCalibrator::collectImages() {
    // create New Folder 
    mkdir(workingFold.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cout << "Press 'X' to capture a new image" << endl;
    int key = 0; // default key number 
    int count = 1;
    while (key != 27) {
        for (int id = 0; id < 2; id++) {
            auto& ctx = cameras[id];
            std::lock_guard<std::mutex> guard(ctx.m);
            if (ctx.hasFrame) {
                Mat gray;
                cvtColor(ctx.frameCam, gray, CV_BGR2GRAY);
#define CIRCLE_CALIB
#ifndef CIRCLE_CALIB
                ctx.flags = CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING;
                ctx.circlesResult = findCirclesGrid(ctx.frameCam, _calibrator->getChessboardSize(),ctx.pt,ctx.flags);
#endif
                ctx.flags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK;
                ctx.circlesResult = findChessboardCorners(gray, _calibrator->getChessboardSize(), ctx.pt, ctx.flags);
                
                ctx.cornersImage = ctx.frameCam.clone(); // if corners were founded use this MAT
                if (ctx.circlesResult) {
                    drawChessboardCorners(ctx.cornersImage, _calibrator->getChessboardSize(), ctx.pt, true);
                    imshow("Corners" + to_string(id), ctx.cornersImage);
                    pnts[id] = ctx.pt;
                    ctx.pt.clear();
                } else {
                    imshow("Corners" + to_string(id), ctx.frameCam);
                }
                
                //testHomography();
                ctx.hasFrame = false;
            }
            if (key == 'x' && cameras[0].circlesResult && cameras[1].circlesResult) {
                string name = workingFold + "/cam_" + to_string(count++) + ".jpg";
                ctx.imagelist.push_back(name);
                imwrite(name, ctx.frameCam);
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
    } else {
        imshow("Corners", image);
    }
    waitKey(0);
}

int TwoCamerasCalibrator::getPoints(vector<Points2f> *imagePoints, vector<Points3f> &objectPoints) {
    if (imagelist.size() == 0 || imagelist.size() % 2 != 0) {
        cout << "Error: the image list is empty or contatins odd (non-even) number of element";
        return -1;
    }
    int nimages = (int) imagelist.size();
    int nmb = 0;
    for (int i = 0; i < nimages; i++) {
        const string &filename = imagelist[i];
        Mat img = imread(filename, 0);
        if (img.empty()) {
            cout << "Image is empty!" << endl;
            break;
        }
        int id = i % 2; // check the id of camera 
        cout << i + 1 << ") " << id << " - " << img.size() << " - " << filename << endl;
        imagePoints[id].push_back(collectPoints(img));
        if (id == 0) {
            objectPoints.push_back(_calibrator->collectObjectPoints());
            nmb++;
        }
    }
    return nmb;
}

// Checks if a matrix is a valid rotatiosn matrix.

bool isRotationMatrix(Mat &R) {
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3, 3, shouldBeIdentity.type());
    return norm(I, shouldBeIdentity) < 1e-6;
}

// Convert a Rotation Matrix to Euler Angles.

Vec3f rotationMatrixToEulerAngles(Mat &R) {
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * (180 / 3.14);
        y = atan2(-R.at<double>(2, 0), sy) * (180 / 3.14);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * (180 / 3.14);
    } else {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1)) * (180 / 3.14);
        y = atan2(-R.at<double>(2, 0), sy) * (180 / 3.14);
        z = 0;
    }
    return Vec3f(x, y, z);
}

int TwoCamerasCalibrator::stereoCalibrateRectificate(Size boardSize, float squareSize) {
    vector<Points2f> imagePoints[2];
    vector<Points3f> objectPoints;
    Mat cameraMatrix[2];
    Mat distCoeffs[2];
    Mat R, T, E, F;
    // Get the number of images pairs where pattern was detected
    int nimages = getPoints(imagePoints, objectPoints);
    if (nimages == 0) {
        cout << "No images was detected" << endl;
        return -1;
    }
    cout << "There were " << nimages << " pairs of image detected." << endl;
    Mat img = imread(imagelist[0], IMREAD_GRAYSCALE);
    Size imageSize = img.size(); // size of the first camera   
    cout << "...Stereo calibration is loading and running..." << endl;
    
    
    // Get all parameters from single camera's claibration
    for (int id = 0; id < 2; id++) {
        imagePoints[id].resize(nimages);
        cameraMatrix[id] = cameras[id].intrinsicsMatrix;
        distCoeffs[id] = cameras[id].distortions;
    }
    
    
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
            cameraMatrix[0], distCoeffs[0],
            cameraMatrix[1], distCoeffs[1],
            Size(), R, T, E, F,
            CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_ASPECT_RATIO,
            TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));

    cout << "Stereo Calibrate done with RMS error = " << rms << endl;
    // ---------------------- (BEGIN) CALIBRATION QUALITY CHECK ------------------------------------
    double err = 0;
    int nPoints = 0;
    vector<Vec3f> lines[2];
    int i, j, k; // iterators
    cout << "ImagePoints(Couple images): " << imagePoints[0].size() << endl;
    for (i = 0; i < nimages; i++) { // 22 go through pairs of camera's image
        int numberPoints = (int) imagePoints[0][i].size(); // 44 
        Mat imgPoints[2];
        for (k = 0; k < 2; k++) { // for each of camera's make an undistort && computeCorrespondEpilines  
            imgPoints[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgPoints[k], imgPoints[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]); // Observed point coord. -> ideal points coord.           
            computeCorrespondEpilines(imgPoints[k], k + 1, F, lines[k]);
        }
        for (j = 0; j < numberPoints; j++) {
            double errij = fabs(imagePoints[0][i][j].x * lines[1][j][0] +
                    imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
                    fabs(imagePoints[1][i][j].x * lines[0][j][0] +
                    imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
            err = err + errij;
        }
        nPoints = nPoints + numberPoints;
    }
    cout << "Average epipolar error = " << err / nPoints << endl;
    // ---------------------- (END) CALIBRATION QUALITY CHECK --------------------------------------
    // ---------------------- (BEGIN) Save Intrinsic parameters (Matrix and Distortion coeff) ------------------------------------  
    FileStorage fs(workingFold + "/instrisics.yml", FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "M0" << cameraMatrix[0] << "D0" << distCoeffs[0]
           << "M1" << cameraMatrix[1] << "D1" << distCoeffs[1];
        fs.release();
    } else {
        cout << "Error: can not save the intrinsic parameters\n";
    }
    // ---------------------- (END) Save Intrinsic parameters ------------------------------------ 
    // ---------------------- (BEGIN) Save Extrinsic parameters ------------------------------------ 
    // Find a feature's position in space from the 2 images I get.
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    cout << "IMAGE SIZE = " << imageSize << endl;
    stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
            imageSize, R, T, R1, R2, P1, P2, Q,
            CALIB_ZERO_DISPARITY,1,imageSize,
            &validRoi[0], &validRoi[1]);
    fs.open(workingFold + "/extrisics.yml", FileStorage::WRITE);
    cout << "ROTATION: " << rotationMatrixToEulerAngles(R) << endl; // get the angle from rotation matrix 
    cout << "TRANSLATION: " << T.t() << endl; // get the angle from rotation matrix 
    if (fs.isOpened()) {
        fs << "R" << R << "T" << T
           << "R1" << R1 << "R2" << R2 
           << "P1" << P1 << "P2" << P2 << "Q" << Q << "F" << F;
    } else
        cout << "Error: Extrinsic parameters can't be save" << endl;
    // ---------------------- (END) Save Extrinsic parameters ------------------------------------ 
    return EXIT_SUCCESS;
}

int TwoCamerasCalibrator::mergeImages() {
    Mat frameCamRotated;
    int key = 0;
    while (key != 27) {
        for (int id = 0; id < 2; id++){
            auto& ctx = cameras[id];
            if (ctx.hasFrame) {
                std::string winName = "FrameCam " + to_string(id);
                cout << "Size: id-" + to_string(id) << " " << ctx.frameCam.size() << endl;
                imshow(winName, ctx.frameCam);
                ctx.hasFrame = false;
            }
        }
        key = waitKey(1);
        if (cameras[1].frameCam.size().height > 0 && cameras[0].frameCam.size().height > 0) {
            DualCameraAligner dca(cameras[0].intrinsicsMatrix, cameras[1].intrinsicsMatrix, R);
            frameCamRotated = dca.align(cameras[0].frameCam); // camera which is cover the main frame(align TV camera)
            DualCameraMerger dcm;
            Mat result = dcm.merge(frameCamRotated, cameras[1].frameCam);
            if (result.cols > 0) {
                imshow("Result", result);
            }
        }
    }
    return EXIT_SUCCESS;
}

void TwoCamerasCalibrator::Help() {
    cout << "1. Collect images from both cameras(TV,IK) for StereoCalibration!" << endl;
    cout << "2. Stereo Calibration" << endl;
    cout << "3. Frames merging" << endl;
    cout << "Choose the option: ";
}

void TwoCamerasCalibrator::imageRetriever(string &folder) {
    vector<String> filename;
    glob(folder, filename);
    sort(filename.begin(), filename.end(), [](const string& s1, const string & s2) {
        if (s1.length() < s2.length())
            return true;
        if (s2.length() < s1.length())
            return false;
        else
            return (s1 < s2);
    });
    cout << "The numbers of frames: " << filename.size() << endl;
    for (int i = 0; i < filename.size(); i++) {
        imagelist.push_back(filename[i]);
    }
}

int TwoCamerasCalibrator::twoCamerasCalibration() {
    int option = 1; // collect images by default       
    Help();
    cin >> option; // chose option
    if (loadCalibrations(option) == EXIT_FAILURE) { // check for right input
        return EXIT_FAILURE;
    }
    switch (option) {
        case 1: {
            startCameras();
            collectImages();
            stopCameras();
            break;
        }
        case 2: {
            Size boardSize(BOARD_SIZE_WIDHT, BOARD_SIZE_HEIGHT);
            string folder = workingFold + "/*.jpg";
            imageRetriever(folder);
            return stereoCalibrateRectificate(boardSize, SQUARE_SIZE);
            break;
        }
        case 3: {
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