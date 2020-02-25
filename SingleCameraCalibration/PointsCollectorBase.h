/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PointsCollectorBase.h
 * Author: ilya
 *
 * Created on 30 октября 2018 г., 11:38
 */

#ifndef POINTSCOLLECTORBASE_H
#define POINTSCOLLECTORBASE_H
#include <vector>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;
class PointsCollectorBase
{
public:
    virtual ~PointsCollectorBase() {};
    virtual void collectKeyPoints(Mat &image, vector<Point2f> &framePoints,vector<Point3f> &basePoints) = 0;
};


#endif /* POINTSCOLLECTORBASE_H */

