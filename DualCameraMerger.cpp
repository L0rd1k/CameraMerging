/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DualCameraMerger.cpp
 * Author: ilya
 *
 * Created on 21 ноября 2018 г., 11:39
 */

#include "DualCameraMerger.h"

DualCameraMerger::DualCameraMerger() 
{
}

Mat DualCameraMerger::merge(Mat img1, Mat img2) // img1- TV img-IK
{
    if(img1.size().height == 0 || img2.size().height == 0) {
        return Mat();        
    }
        

    Mat edges, result, buffer[3];
    
    int w = std::max(img1.size().width, img2.size().width);
    int h = std::max(img1.size().height, img2.size().height);
    Size largeSize(w, h);
    Mat largerImage(largeSize, edges.type(), Scalar(0));
    Mat large1(largeSize, img1.type());
    Mat large2(largeSize, img2.type());
    Mat roi1(large1, Rect(0, 0, img1.cols, img1.rows));
    Mat roi2(large2, Rect(0, 0, img2.cols, img2.rows));
    img1.copyTo(roi1);
    img2.copyTo(roi2);
    
    cvtColor(large1, edges, COLOR_BGR2GRAY);
    GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
    Canny(edges, edges, 0, 30, 3);
    split(large2, buffer);
    edges.copyTo(largerImage(Rect(0, 0, edges.cols, edges.rows)));
    bitwise_or(buffer[2], largerImage, buffer[2]);
    cv::merge(buffer, 3, result);
      
//        resize(img1,img1,img2.size()); // resize TV cameras, to IK size 
//        cvtColor(img1,edges, COLOR_BGR2GRAY);
//        
//        GaussianBlur(edges,edges, Size(7,7),1.5,1.5); // Image denoising 
//        Canny(edges,edges,0,30,3); // Canny to TV camera
//        
//        split(img2, buffer); // Divide IK's multi-channel array into several single-channel arrays.        
//        Mat largerImage(Size(img2.size().width, img2.size().height), edges.type()); // canvas for transformed TV image
//        largerImage = Scalar(0); // initializtion of empty Mat        
//        edges.copyTo(largerImage(Rect(0,0,edges.cols, edges.rows))); // copy Mat edges to Mat largerImage        
//        
//        //Mat largerTest;
//        //edges.copyTo(largerTest,largerImage.size());
//        
//        cout << buffer[2].size() << endl;
//        cout << largerImage.size() << endl;
//
//        bitwise_or(buffer[2], largerImage, buffer[2]); // 2nd channel of IK image 
//        cv::merge(buffer,3,result);
    return result;
}


DualCameraMerger::~DualCameraMerger() {
}

