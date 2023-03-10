#include <iostream>
#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

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

int calibrateAndSave(std::vector<std::vector<cv::Vec3f>> &pointList, std::vector<std::vector<cv::Point2f>> &cornerList, cv::Size imgSize){
    cv::Mat cameraMatrix  = cv::Mat::eye(3,3,CV_64F);
    cameraMatrix.at<double>(0,2) = imgSize.width/2;
    cameraMatrix.at<double>(1,2) = imgSize.height/2;
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    printDoubleMatrix(cameraMatrix);
    printDoubleMatrix(distCoeffs);
    double avgErr = cv::calibrateCamera(pointList, cornerList, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO);
    printDoubleMatrix(cameraMatrix);
    printDoubleMatrix(distCoeffs);
    std::cout << "overall re-projection error: " << avgErr << std::endl;
    saveIntrinsic(cameraMatrix, distCoeffs);
    return 0;
}

void printDoubleMatrix(cv::Mat &matrix){
    for(int i=0; i<matrix.rows; i++){
        for(int j=0; j<matrix.cols; j++){
            std::cout<< matrix.at<double>(i,j) << " ";
        }
        std::cout<<std::endl;
    }
}

void saveIntrinsic(cv::Mat &cameraMatrix, cv::Mat &distCoeffs){
        std::string fileName = "intrinsicParameters.xml";
        std::string cameraMatrixName = "cameraMatrix";
        std::string distCoeffsName = "distCoeffs";
        cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
        fs<< cameraMatrixName << cameraMatrix;
        fs<< distCoeffsName << distCoeffs;
        fs.release();
        return;
}