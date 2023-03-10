#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>

#include "filters.h"

int grayscale(cv::Mat &src, cv::Mat &dst){
    dst = cv::Mat::zeros(src.size(), CV_8U);
    for(int i=0;i<src.rows; i++){
        uchar *drptr = dst.ptr<uchar>(i);
        cv::Vec3b *rptr = src.ptr<cv::Vec3b>(i);
        for(int j=0;j<src.cols;j++){
            drptr[j] = (int)(0.114*rptr[j][0]+0.587*rptr[j][1]+0.299*rptr[j][2]); //coefficience from opencv color conversion
        }
    }
    return 0;
}