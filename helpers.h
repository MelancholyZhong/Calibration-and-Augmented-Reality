#ifndef HELPERS
#define HELPERS
/**
    Calibration and Augmented Reality
    Created by Yao Zhong for CS 5330 Computer Vision Spring 2023

    helperfunctions for the project
*/


//Helper function that convert a color image to  grayscale image
int grayscale( cv::Mat &src, cv::Mat &dst );

//Togenerate a list of 3d points of the chessboard eg. (0,0,0),(0,-1,0)
int generatePointSet(std::vector<cv::Vec3f> &pointSet, cv::Size patternSize);

//Togenerate the 4 outside 3D points of the chessboard
int generateOutSidePoints(std::vector<cv::Vec3f> &outSidePoints, cv::Size patternSize);

//Generate the corner points of a ArUco label in 3d, eg(0,0,0).
void generateArucoPoints(std::vector<cv::Vec3f> &arucoPointSet);

//calibrate the camera and save the intrinsic parameters
int calibrateAndSave(std::vector<std::vector<cv::Vec3f>> &pointList, std::vector<std::vector<cv::Point2f>> &cornerList, cv::Size imgSize);

//helper function used to print a matrix
void printDoubleMatrix(cv::Mat &matrix);

//save the given matrixs into the XML file
void saveIntrinsic(cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

//render the virtual object to the image
void renderObject(cv::Mat &frame, cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

//given the detected corners, draw the harris corner
void drawHarrisCorner(cv::Mat &frame, cv::Mat &corners);

#endif