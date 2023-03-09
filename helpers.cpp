#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#include "helpers.h"

int generatePointSet(std::vector<cv::Vec3f> &pointSet, cv::Size patternSize){
    for(int i=0; i<patternSize.height; i++){
        for(int j=0; j< patternSize.width; j++){
            cv::Vec3f point(j,-1*i, 0);
            pointSet.push_back(point);
        }
    }
    return 0;
}