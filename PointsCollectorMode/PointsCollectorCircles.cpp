/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PointsCollectorCircles.cpp
 * Author: ilya
 * 
 * Created on 25 октября 2018 г., 16:25
 */

#include "PointsCollectorCircles.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

PointsCollectorCircles::PointsCollectorCircles(int x, int y, int size, bool isAsymmetric) 
{
    this->chessboardSize = Size(x,y); //[4 x 11
    this->squareSize = size; // 44 mm
    this->isAsymmetric = isAsymmetric; // the type of board(Assymetrical or Symmetrical)
}

PointsCollectorCircles::~PointsCollectorCircles() 
{
}

void PointsCollectorCircles::collectKeyPoints(Mat& image, vector<Point2f>& framePoints, vector<Point3f>& basePoints)
{
    basePoints = collectObjectPoints(); // how the main points located in real world;
    framePoints = collectFramePoints(image); // vector of, main frame points we get from orignal image(x,y)
}

vector<Point2f> PointsCollectorCircles::collectFramePoints(Mat &image)
{
    vector<Point2f> pts;
    int flags; // calibration flag
    if(isAsymmetric) { // choose the correct flag for calibration pattern
        flags = CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING; // flag for asymmetric pattern
        //  CALIB_CB_CLUSTERIN - uses a special algorithm for grid detection. It is more robust to perspective distortions.
    } else {
        flags = CALIB_CB_SYMMETRIC_GRID; // flag for symmetric pattern
    }
    bool circlesResult = findCirclesGrid(image, chessboardSize, pts, flags); // find if the circle's pattern exist on the image
    if (!circlesResult) {// in case patern wasn't detected
            pts.clear(); // clear the vector of frame points
    }  
    return pts;
}

vector<Point3f> PointsCollectorCircles::collectObjectPoints()
{
    vector<Point3f> tmpObjectPoints;
    float currentShift = 0;
    int heightSize = squareSize; // 38mm
    if(this->isAsymmetric)
    {
        heightSize /=2; // 19 mm
    }
    int i = 0;
    for(int y = 0; y < chessboardSize.height; y++) { // 11 itterations
        if(this->isAsymmetric) {
            if(i % 2 == 0) {
                currentShift = 0; 
            }
            else
            {
                currentShift = squareSize / 2;
            }
        }
        i++; 
        for(int x = 0; x < chessboardSize.width; x++)
        {
            Point3f tmpObjectPoint(x * squareSize + currentShift, y * heightSize, 0); //
            tmpObjectPoints.push_back(tmpObjectPoint);
        }
    }
    return tmpObjectPoints;
}

Size PointsCollectorCircles::getChessboardSize()
{
    return chessboardSize;
}