/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CalibratorBase.h
 * Author: ilya
 *
 * Created on 25 октября 2018 г., 15:29
 */

#ifndef CALIBRATORBASE_H
#define CALIBRATORBASE_H

#include <vector>

using namespace std;
using namespace cv;

class CalibratorBase {
public:
    virtual ~CalibratorBase();    
    virtual vector<Point3f> collectObjectPoints() = 0;
    virtual vector<Point2f> collectFramePoints(Mat &image) = 0;
    virtual Size getChessboardSize() = 0;
};

#endif /* CALIBRATORBASE_H */

