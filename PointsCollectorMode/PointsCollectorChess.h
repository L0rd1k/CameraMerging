/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PointsCollectorChess.h
 * Author: ilya
 *
 * Created on 25 октября 2018 г., 16:25
 */

#ifndef POINTSCOLLECTORCHESS_H
#define POINTSCOLLECTORCHESS_H

#include "CalibratorBase.h"
#include "PointsCollectorBase.h"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

class PointsCollectorChess :  public PointsCollectorBase, public CalibratorBase {
public:
    PointsCollectorChess(int x = 9, int y = 6, int size = 25);
    virtual ~PointsCollectorChess();
    
    virtual void collectKeyPoints(Mat &image, vector<Point2f> &framePoints,vector<Point3f> &basePoints);
    virtual vector<Point3f> collectObjectPoints();
    virtual vector<Point2f> collectFramePoints(Mat &image);
    virtual Size getChessboardSize();
     
private:
    Size chessboardSize;
    float squareSize;

};

#endif /* POINTSCOLLECTORCHESS_H */

