/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FastUndistort.cpp
 * Author: ilya
 * 
 * Created on 31 октября 2018 г., 13:53
 */

#include "FastUndistort.h"

FastUndistort::FastUndistort(InputArray cameraMatrix,InputArray distCoeffs)
{
    p_cameraMatrix = cameraMatrix.getMat();
    p_distCoeffs = distCoeffs.getMat();
}

FastUndistort::~FastUndistort() 
{
}

void FastUndistort::undistort(InputArray p_src, OutputArray p_dst)
{
    Mat src = p_src.getMat(), cameraMatrix = p_cameraMatrix;
    Mat distCoeffs = p_distCoeffs;
    p_dst.create(src.size(), src.type());
    Mat dst = p_dst.getMat();
    CV_Assert(dst.data != src.data);
    
    Mat_<double> A, Ar, I = Mat_<double>::eye(3,3);
    p_cameraMatrix.convertTo(A,CV_64F);
    if(!distCoeffs.empty())
    {
        distCoeffs = Mat_<double>(distCoeffs);
    }
    else
    {
        distCoeffs.create(5,1,CV_64F);
        distCoeffs = 0.;
    }
    A.copyTo(Ar);
    double v0 = Ar(1,2);
    bool newRectifyMap = (src.rows != p_height) || (src.cols != p_width);
    
    if(newRectifyMap)
    {
        map1 = Mat(src.rows, src.cols, CV_16SC2);
        map2 = Mat(src.rows, src.cols, CV_16UC1);
        initUndistortRectifyMap(A, distCoeffs, I, Ar, map1.size(), map1.type(), map1, map2);
        p_height = src.rows;
        p_width = src.cols;
    }
    
    remap(src, dst, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
}

