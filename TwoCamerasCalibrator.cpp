/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TwoCamerasCalibrator.cpp
 * Author: ilya
 * 
 * Created on 7 ноября 2018 г., 12:27
 */

#include "TwoCamerasCalibrator.h"

TwoCamerasCalibrator::TwoCamerasCalibrator() 
{
}

TwoCamerasCalibrator::~TwoCamerasCalibrator() 
{
    
}

int twoCamerasCalibration()
{
    int option = 1 ; // collect images by default       
    cout << "1. Collect images for StereoCalibration " << endl;
    cout << "2. " << endl;
    cout << "3. " << endl; 
    cout << "Choose the option: ";
    cin >> option;   
    
    switch (option)
    {
        case 1: 
        {
            break;
        }
        case 2:
        {
            break;
        }
        case 3:
        {
            break;
        }
        default:
            return 1; // return EXIT_FAILURE
            
    }
    return 0; // EXIT_SUCCESS
}