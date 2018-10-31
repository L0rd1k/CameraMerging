/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PointsCollectorCircles.h
 * Author: ilya
 *
 * Created on 25 октября 2018 г., 16:25
 */

#ifndef POINTSCOLLECTORCIRCLES_H
#define POINTSCOLLECTORCIRCLES_H
#include "PointsCollectorBase.h"
#include "CalibratorBase.h"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

class PointsCollectorCircles : public PointsCollectorBase, public CalibratorBase 
{
public:
    PointsCollectorCircles(int x = 4, int y = 11, int size = 44, bool isAsymmetric = true);
    virtual ~PointsCollectorCircles();
    void collectKeyPoints(Mat &image, vector<Point2f> &framePoints,vector<Point3f> &basePoints) override;
    vector<Point3f> collectObjectPoints() override;
    vector<Point2f> collectFramePoints(Mat &image) override;
    Size getChessboardSize() override;
private:
    Size chessboardSize;
    float squareSize;
    bool isAsymmetric;  
};

#endif /* POINTSCOLLECTORCIRCLES_H */

