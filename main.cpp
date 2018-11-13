/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: ilya
 *
 * Created on 25 октября 2018 г., 10:32
 */

#include <cstdlib>
#include "SingleCalibration.h"
#include "TwoCamerasCalibrator.h"
#include "Undistort.h"
using namespace std;
using namespace cv;
/*
 * 
 */
int main(int argc, char** argv) 
{
    //cameraCalibration(); // 1
    //testUndistort(); // 2
    
    TwoCamerasCalibrator tcc;
    tcc.twoCamerasCalibration();
    
    return 0;
}

