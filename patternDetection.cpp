/**
    Calibration and Augmented Reality
    Created by Yao Zhong for CS 5330 Computer Vision Spring 2023

    robust feature dectetion main function
*/
#include <iostream>
#include <string>
#include <filesystem>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types_c.h>

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "helpers.h"

int main(int argc, char *argv[]) {
        cv::VideoCapture *capdev;

        // open the video device
        capdev = new cv::VideoCapture(0);
        if( !capdev->isOpened() ) {
                printf("Unable to open video device\n");
                return(-1);
        }

        // get some properties of the image
        cv::Size refS( (int) capdev->get(cv::CAP_PROP_FRAME_WIDTH ),
                       (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
        printf("Expected size: %d %d\n", refS.width, refS.height);

        cv::namedWindow("Video", 1); // identifies a window
        cv::Mat frame;

        //image capturing settings
        int captured = 0;
        std::__fs::filesystem::create_directory("./captured");

        //We need to save a frame of SURF feature, so that can be used for the futher matching
        int mode = 1; // 0 for harris-corner 1, for Surf and object render, 2 for matching
        bool haveTrained = false;
        std::vector<cv::KeyPoint> trainedKeypoints;
        cv::Mat trainedDescriptors;

        //Read the intrinsic parameters into the program
        std::string fileName = "intrinsicParameters.xml";
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        cv::FileStorage fs(fileName, cv::FileStorage::READ);
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;

        for(;;) {
                *capdev >> frame; // get a new frame from the camera, treat as a stream
                if( frame.empty() ) {
                  printf("frame is empty\n");
                  break;
                }
                
                //call the grascale function to get the grayscale image
                cv::Mat gray;
                grayscale(frame, gray);

                //container variables for the coners and keypoints
                cv::Mat corners = cv::Mat::zeros(frame.size(), CV_32FC1);
                std::vector<cv::KeyPoint> keypoints;
                cv::Mat descriptors;

                if(mode==0){
                    //detect harriscorner and draw it out
                    cv::cornerHarris(gray, corners, 2, 3, 0.04);
                    drawHarrisCorner(frame, corners);
                }else{
                    //define the SURF feature detector
                    int minHessian = 5000;
                    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );
                    //detect the features and compute the discripters
                    detector->detectAndCompute( gray, cv::noArray(), keypoints, descriptors );
                    if(mode == 1){
                        cv::drawKeypoints( frame, keypoints, frame);
                    }else if(mode ==2){
                        //Note: this is an unsuccessful attempt for the extension

                        //define the matcher for SURF features
                        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
                        std::vector< std::vector<cv::DMatch> > knn_matches;
                        //Match between the current frame and the saved feature
                        matcher->knnMatch( descriptors, trainedDescriptors, knn_matches, 2 );
                        //To get the good matches
                        const float ratio_thresh = 0.7f;
                        std::vector<cv::DMatch> good_matches;
                        for (int i = 0; i < knn_matches.size(); i++){
                            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
                                good_matches.push_back(knn_matches[i][0]);
                            }
                        }

                        //only when 4 or more features matched(which is required by the solvePNPRansac), the camera position estimate will happen
                        if(good_matches.size()>=4){
                            std::vector<cv::Vec3f> objPoints;
                            std::vector<cv::Point2f> imgPoints;
                            //try to estimate the 3D points by adding "0" as the Z axes value
                            for(int i = 0; i < good_matches.size(); i++){
                                int queryIdx = good_matches[i].queryIdx;
                                int trainIdx = good_matches[i].trainIdx;
                                cv::Vec3f point_1(trainedKeypoints[trainIdx].pt.x,trainedKeypoints[trainIdx].pt.y ,0);
                                objPoints.push_back(point_1);
                                imgPoints.push_back(keypoints[queryIdx].pt);
                            }

                            //use solvePNPRansac to estimate the camera position
                            cv::Mat rvecs, tvecs;
                            cv::solvePnPRansac(objPoints, imgPoints, cameraMatrix, distCoeffs, rvecs, tvecs);
                            cv::solvePnPRefineLM(objPoints, imgPoints, cameraMatrix, distCoeffs, rvecs, tvecs);

                            //This is the attempt to draw the three axes of the image
                            std::vector<cv::Vec3f> outSidePoints;
                            outSidePoints.push_back(objPoints[0]); //origin point
                            cv::Vec3b point1(objPoints[0][0]+10, objPoints[0][1], 0); //y axis ending point
                            outSidePoints.push_back(point1);
                            cv::Vec3b point2(objPoints[0][0], objPoints[0][1]+10, 0); // x axis ending point
                            outSidePoints.push_back(point2);
                            cv::Vec3b point3(objPoints[0][0], objPoints[0][1], 10); //z axis ending point
                            outSidePoints.push_back(point3);

                            //project the points accroding to rvecs and tvecs, and then draw the lines
                            std::vector<cv::Point2f> outSideImgPoints;
                            cv::projectPoints(outSidePoints, rvecs, tvecs, cameraMatrix, distCoeffs, outSideImgPoints);
                            cv::Scalar color(149,86,17);
                            cv::line(frame, outSideImgPoints[0], outSideImgPoints[1], color, 3);
                            cv::line(frame, outSideImgPoints[0], outSideImgPoints[2], color, 3);
                            cv::line(frame, outSideImgPoints[0], outSideImgPoints[3], color, 3);
                        }
                    }
                }

                cv::imshow("Video", frame);

                // see if there is a waiting keystroke
                char key = cv::pollKey();
                if( key == 'q') {
                    break;
                }else if (key == 's'){
                        captured += 1;
                        std::string capturedStr = std::to_string(captured);
                        std::string fileName = "./captured/robustFeature_" + capturedStr + ".jpg";
                        cv::imwrite(fileName,frame);
                        //save the frame for latter matching step
                        if(mode == 1){
                            haveTrained = true;
                            trainedKeypoints = keypoints;
                            trainedDescriptors = descriptors;
                        }
                }else if(key == 't'){
                    //switch between harris corner and SURF feature
                    mode == 0 ? mode = 1 : mode = 0;
                }else if(key == 'r' && haveTrained){
                    //swich between feature dectetion and object projection
                    mode == 2 ? mode = 1 : mode = 2;
                }
        }

        delete capdev;
        return(0);
}