/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PointsCollectorChess.cpp
 * Author: ilya
 * 
 * Created on 25 октября 2018 г., 16:25
 */

#include "PointsCollectorChess.h"
#include <opencv2/highgui.hpp>

PointsCollectorChess::PointsCollectorChess(int x, int y, int size) 
{
    chessboardSize = Size(x, y);
    squareSize = size;
}

PointsCollectorChess::~PointsCollectorChess() {
}


vector<Point2f> PointsCollectorChess::collectFramePoints(Mat &image)
{
    vector<Point2f> pts;
    
    //cout <<"!!!!"<<image.size() << endl;
    
    for(double p = -4; p <= 4; p++) // scale image until chessboard would not be detected
    {
        Mat imageResized;
        double scale = pow(1.5, p);
        resize(image, imageResized, Size(), scale, scale);
        // Finds the positions of internal corners of the chessboard.
        bool chessboardResult = findChessboardCorners(imageResized, chessboardSize, pts, 
                CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_ADAPTIVE_THRESH);
        
        imshow("image", imageResized);
        waitKey(100);
        cout << chessboardResult << endl;
        int key = waitKey(0);
        
        if(chessboardResult) // if chessboard corners were detecting
        {
            for(auto &p: pts)
            {
                p.x /= scale;
                p.y /= scale;
            }
            // Refines the corner locations.
            cornerSubPix(image, pts, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            break;
        }
        else
        {
            pts.clear();
        }
    }
    return pts;
}

Size PointsCollectorChess::getChessboardSize()
{
    return chessboardSize;
}

vector<Point3f> PointsCollectorChess::collectObjectPoints()
{
    vector<Point3f> tmpObjectPoints;
    Point3f shift(0,0,0);
    for(int y = 0; y < chessboardSize.height; y++)
    {
        for(int x = 0; x < chessboardSize.width; x++)
        {
            Point3f tmpObjectPoint(x*squareSize, y*squareSize, 0);
            tmpObjectPoints.push_back(tmpObjectPoint + shift);
        }
    }
    return tmpObjectPoints;
}

void PointsCollectorChess::collectKeyPoints(Mat& image, vector<Point2f> &framePoints, vector<Point3f> &basePoints) 
{
    basePoints = collectObjectPoints();
    framePoints = collectFramePoints(image);
}