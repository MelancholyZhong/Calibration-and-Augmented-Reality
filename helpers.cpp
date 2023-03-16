#include <iostream>
#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "helpers.h"

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

int generatePointSet(std::vector<cv::Vec3f> &pointSet, cv::Size patternSize){
    for(int i=0; i<patternSize.height; i++){
        for(int j=0; j< patternSize.width; j++){
            cv::Vec3f point(j,-1*i, 0);
            pointSet.push_back(point);
        }
    }
    return 0;
}

int generateOutSidePoints(std::vector<cv::Vec3f> &outSidePoints, cv::Size patternSize){
    cv::Vec3f point1(-1,1,0);
    outSidePoints.push_back(point1);
    cv::Vec3f point2(patternSize.width,1,0);
    outSidePoints.push_back(point2);
    cv::Vec3f point3(-1,-1*patternSize.height,0);
    outSidePoints.push_back(point3);
    cv::Vec3f point4(patternSize.width,-1*patternSize.height,0);
    outSidePoints.push_back(point4);
    return 0;
}

void generateArucoPoints(std::vector<cv::Vec3f> &arucoPointSet){
    cv::Vec3f point1(0,0,0);
    arucoPointSet.push_back(point1);
    cv::Vec3f point2(1,0,0);
    arucoPointSet.push_back(point2);
    cv::Vec3f point3(1,-1,0);
    arucoPointSet.push_back(point3);
    cv::Vec3f point4(0,-1,0);
    arucoPointSet.push_back(point4);
    return;
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

void drawCube(cv::Mat &frame, cv::Point3f position, cv::Scalar color, cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &cameraMatrix, cv::Mat &distCoeffs){
    std::vector<cv::Point3f> objectPoints{cv::Point3f(position.x,position.y,position.z), cv::Point3f(position.x,position.y,position.z+1), 
                    cv::Point3f(1+position.x,position.y,position.z), cv::Point3f(1+position.x,position.y,position.z+1),
                    cv::Point3f(1+position.x,-1+position.y,position.z), cv::Point3f(1+position.x,-1+position.y,position.z+1),
                    cv::Point3f(position.x,-1+position.y,position.z), cv::Point3f(position.x,-1+position.y,position.z+1)};
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints);
    for(int i=0; i<4; i++){
        cv::line(frame, imagePoints[2*i], imagePoints[(2*i+2)%8], color, 3);
        cv::line(frame, imagePoints[2*i+1], imagePoints[(2*i+2)%8+1], color, 3);
        cv::line(frame, imagePoints[2*i], imagePoints[2*i+1], color, 3);
    }
    return;
}
   
void renderObject(cv::Mat &frame, cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &cameraMatrix, cv::Mat &distCoeffs){
    std::vector<cv::Point3f> trunck{cv::Point3f(3,-3,0), cv::Point3f(3,-3,1), cv::Point3f(3,-3,2), cv::Point3f(3,-3,3)};
    std::vector<cv::Point3f> canopy{cv::Point3f(3,-4,3), cv::Point3f(4,-4,3), cv::Point3f(2,-3,3), cv::Point3f(4,-3,3),cv::Point3f(5,-3,3),
                                                            cv::Point3f(2,-2,3), cv::Point3f(3,-2,3), cv::Point3f(2,-3,4), cv::Point3f(3,-3,4),cv::Point3f(3,-3,5)};
    std::vector<cv::Point3f> heart{cv::Point3f(7,-6,0), cv::Point3f(7,-6,1), cv::Point3f(7,-6,2), 
                                                        cv::Point3f(6,-6,1),  cv::Point3f(6,-6,2), cv::Point3f(6,-6,3),
                                                        cv::Point3f(5,-6,2),  cv::Point3f(8,-6,1), cv::Point3f(8,-6,2),
                                                        cv::Point3f(8,-6,3),  cv::Point3f(9,-6,2)};
    cv::Scalar trunckColor(66,92,114);
    cv::Scalar green(34,139,40);
    cv::Scalar pink(172, 142, 252);
    for(int i=0; i<trunck.size(); i++){
        auto position = trunck[i];
        drawCube(frame, position, trunckColor, rvecs, tvecs, cameraMatrix, distCoeffs);
    }
    for(int i=0; i<canopy.size(); i++){
        auto position = canopy[i];
        drawCube(frame, position, green, rvecs, tvecs, cameraMatrix, distCoeffs);
    }
    for(int i=0; i<heart.size(); i++){
        auto position = heart[i];
        drawCube(frame, position, pink, rvecs, tvecs, cameraMatrix, distCoeffs);
    }
    return;
}

void drawHarrisCorner(cv::Mat &frame, cv::Mat &corners){
        cv::Mat corners_norm, corners_norm_scaled;
        int threshold = 180;
        cv::Scalar color(255,0,0);
        cv::normalize( corners, corners_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
        cv::convertScaleAbs( corners_norm, corners_norm_scaled );
        for(int i=0; i<frame.rows; i++){
            uchar *rptr = corners_norm_scaled.ptr<uchar>(i);
            for(int j=0; j<frame.cols; j++){
                if(rptr[j] > threshold){
                    cv::circle(frame, cv::Point(j, i), 10, color, 3);
                }
            }
        }
        return;
}