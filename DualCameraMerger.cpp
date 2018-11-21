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

Mat DualCameraMerger::merge(Mat img1, Mat img2)
{
    Mat edges, result, buffer[3];
    
    if(img1.size().height > 0 && img2.size().height)
    {
        cvtColor(img1, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        split(img2, buffer);
        Mat largerImage(Size(img2.size().width, img2.size().height), edges.type());
        largerImage = Scalar(0);
        edges.copyTo(largerImage(Rect(0, 0, edges.cols, edges.rows)));
        bitwise_or(buffer[2], largerImage, buffer[2]);
        cv::merge(buffer, 3, result);
    }
    return result;
}


DualCameraMerger::~DualCameraMerger() {
}

