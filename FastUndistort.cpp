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
    Mat src = p_src.getMat();
    //cout << src.size() << endl;
    Mat cameraMatrix = p_cameraMatrix; //matrix coeffs
    Mat distCoeffs = p_distCoeffs; // dist coeffs
    
    p_dst.create(src.size(), src.type()); // create Mat object
    
    Mat dst = p_dst.getMat();  
    CV_Assert(dst.data != src.data); // Checks a condition at runtime and throws exception if it fails.    
    Mat_<double> A, Ar, I = Mat_<double>::eye(3,3); // // create a 3x3 double-precision identity matrix
    p_cameraMatrix.convertTo(A,CV_64F); 
    //cout << distCoeffs << endl;
    if(!distCoeffs.empty()) // Not empty
    {
        distCoeffs = Mat_<double>(distCoeffs);  //convert to double
        //cout << "Double" << distCoeffs << endl;
    }
    else
    {
        distCoeffs.create(5,1,CV_64F);  
        distCoeffs = 0.;
    }
    A.copyTo(Ar); // copy matrix A to Ar
    //cout << Ar << endl;
    //double v0 = Ar(1,2);
    cout << src.rows << " = " << p_height << endl;
    cout << src.cols << " = " << p_width << endl;
    bool newRectifyMap = (src.rows != p_height) || (src.cols != p_width); //compare frame size
    
    if(newRectifyMap)
    {
        map1 = Mat(src.rows, src.cols, CV_16SC2);
        map2 = Mat(src.rows, src.cols, CV_16UC1);
        initUndistortRectifyMap(A, distCoeffs, I, Ar, map1.size(), map1.type(), map1, map2); //Computes the undistortion and rectification transformation map.
        p_height = src.rows;             
        p_width = src.cols;            
    }
    
    remap(src, dst, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
    // the process of taking pixels from one place in the image and locating them in another position in a new image.
    // INTER_LINEAR: The type of interpolation to use for non-integer pixels. This is by default.
    // BORDER_CONSTANT: Default
}

