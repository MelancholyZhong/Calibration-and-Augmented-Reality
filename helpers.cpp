/**
    Calibration and Augmented Reality
    Created by Yao Zhong for CS 5330 Computer Vision Spring 2023

    helperfunctions for the project
*/
#include <iostream>
#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "helpers.h"

//Helper function that convert a color image to  grayscale image
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

//Togenerate a list of 3d points of the chessboard eg. (0,0,0),(0,-1,0)
int generatePointSet(std::vector<cv::Vec3f> &pointSet, cv::Size patternSize){
    for(int i=0; i<patternSize.height; i++){
        for(int j=0; j< patternSize.width; j++){
            cv::Vec3f point(j,-1*i, 0);
            pointSet.push_back(point);
        }
    }
    return 0;
}

//Togenerate the 4 outside 3D points of the chessboard
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

//Generate the corner points of a ArUco label in 3d, eg(0,0,0).
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

//calibrate the camera and save the intrinsic parameters
int calibrateAndSave(std::vector<std::vector<cv::Vec3f>> &pointList, std::vector<std::vector<cv::Point2f>> &cornerList, cv::Size imgSize){
    //initialize the camrea matrix
    cv::Mat cameraMatrix  = cv::Mat::eye(3,3,CV_64F);
    cameraMatrix.at<double>(0,2) = imgSize.width/2;
    cameraMatrix.at<double>(1,2) = imgSize.height/2;
    //initialize the distortion coeffecients
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    printDoubleMatrix(cameraMatrix);
    printDoubleMatrix(distCoeffs);
    //calibrate the camera
    double avgErr = cv::calibrateCamera(pointList, cornerList, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO);
    printDoubleMatrix(cameraMatrix);
    printDoubleMatrix(distCoeffs);
    std::cout << "overall re-projection error: " << avgErr << std::endl;
    //save the matrixs
    saveIntrinsic(cameraMatrix, distCoeffs);
    return 0;
}


//helper function used to print a matrix
void printDoubleMatrix(cv::Mat &matrix){
    for(int i=0; i<matrix.rows; i++){
        for(int j=0; j<matrix.cols; j++){
            std::cout<< matrix.at<double>(i,j) << " ";
        }
        std::cout<<std::endl;
    }
}

//save the given matrixs into the XML file
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

// given the position of position and color, draw a cube on that place, this is the minimum unit of the 3d object. 
void drawCube(cv::Mat &frame, cv::Point3f position, cv::Scalar color, cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &cameraMatrix, cv::Mat &distCoeffs){
    //The eight points of a cube
    std::vector<cv::Point3f> objectPoints{cv::Point3f(position.x,position.y,position.z), cv::Point3f(position.x,position.y,position.z+1), 
                    cv::Point3f(1+position.x,position.y,position.z), cv::Point3f(1+position.x,position.y,position.z+1),
                    cv::Point3f(1+position.x,-1+position.y,position.z), cv::Point3f(1+position.x,-1+position.y,position.z+1),
                    cv::Point3f(position.x,-1+position.y,position.z), cv::Point3f(position.x,-1+position.y,position.z+1)};
    std::vector<cv::Point2f> imagePoints;
    //project the 8 points to the image and draw the lines between them to form a samll cube
    cv::projectPoints(objectPoints, rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints);
    for(int i=0; i<4; i++){
        cv::line(frame, imagePoints[2*i], imagePoints[(2*i+2)%8], color, 3);
        cv::line(frame, imagePoints[2*i+1], imagePoints[(2*i+2)%8+1], color, 3);
        cv::line(frame, imagePoints[2*i], imagePoints[2*i+1], color, 3);
    }
    return;
}

//render the virtual object to the image
void renderObject(cv::Mat &frame, cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &cameraMatrix, cv::Mat &distCoeffs){
    //The list of position will create different shapes, first two is the three
    std::vector<cv::Point3f> trunck{cv::Point3f(3,-3,0), cv::Point3f(3,-3,1), cv::Point3f(3,-3,2), cv::Point3f(3,-3,3)};
    std::vector<cv::Point3f> canopy{cv::Point3f(3,-4,3), cv::Point3f(4,-4,3), cv::Point3f(2,-3,3), cv::Point3f(4,-3,3),cv::Point3f(5,-3,3),
                                                            cv::Point3f(2,-2,3), cv::Point3f(3,-2,3), cv::Point3f(2,-3,4), cv::Point3f(3,-3,4),cv::Point3f(3,-3,5)};

    //This is a heart shape
    std::vector<cv::Point3f> heart{cv::Point3f(7,-6,0), cv::Point3f(7,-6,1), cv::Point3f(7,-6,2), 
                                                        cv::Point3f(6,-6,1),  cv::Point3f(6,-6,2), cv::Point3f(6,-6,3),
                                                        cv::Point3f(5,-6,2),  cv::Point3f(8,-6,1), cv::Point3f(8,-6,2),
                                                        cv::Point3f(8,-6,3),  cv::Point3f(9,-6,2)};
    cv::Scalar trunckColor(66,92,114);
    cv::Scalar green(34,139,40);
    cv::Scalar pink(172, 142, 252);
    //drawing the tree
    for(int i=0; i<trunck.size(); i++){
        auto position = trunck[i];
        drawCube(frame, position, trunckColor, rvecs, tvecs, cameraMatrix, distCoeffs);
    }
    for(int i=0; i<canopy.size(); i++){
        auto position = canopy[i];
        drawCube(frame, position, green, rvecs, tvecs, cameraMatrix, distCoeffs);
    }
    //drawing the heart
    for(int i=0; i<heart.size(); i++){
        auto position = heart[i];
        drawCube(frame, position, pink, rvecs, tvecs, cameraMatrix, distCoeffs);
    }
    return;
}

//given the detected corners, draw the harris corner
void drawHarrisCorner(cv::Mat &frame, cv::Mat &corners){
        cv::Mat corners_norm, corners_norm_scaled;
        int threshold = 180;
        cv::Scalar color(255,0,0);
        //normalize the value at each pixel to 0-255 so that we can pick the threshold as the harris corner
        cv::normalize( corners, corners_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
        cv::convertScaleAbs( corners_norm, corners_norm_scaled );

        //conside the values where over the threshold, and draw them out
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