#include <iostream>
#include <string>
#include <filesystem>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/aruco.hpp>

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

        int captured = 0;
        std::__fs::filesystem::create_directory("./captured");

        std::string fileName = "intrinsicParameters.xml";
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        cv::FileStorage fs(fileName, cv::FileStorage::READ);
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;

        cv::Size patternSize(9,6);
        std::vector<cv::Vec3f> pointSet;
        generatePointSet(pointSet, patternSize);
        std::vector<cv::Vec3f> outSidePoints;
        generateOutSidePoints(outSidePoints, patternSize);
        std::vector<cv::Vec3f> arucoPointSet;
        generateArucoPoints(arucoPointSet);
        
        int mode = 0; //0 for chessboard 1 for ArUco, 0 is defualt

        for(;;) {
                *capdev >> frame; // get a new frame from the camera, treat as a stream
                if( frame.empty() ) {
                  printf("frame is empty\n");
                  break;
                }
                
                cv::Mat gray;
                grayscale(frame, gray);
                cv::Mat rvecs, tvecs;

                if(mode == 0){
                        std::vector<cv::Point2f> cornerSet;
                        bool patternFound = cv::findChessboardCorners(gray, patternSize, cornerSet, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE+ cv::CALIB_CB_FAST_CHECK);

                        if(patternFound){
                                cv::cornerSubPix(gray, cornerSet, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                                cv::solvePnP(pointSet, cornerSet, cameraMatrix, distCoeffs, rvecs, tvecs);
                                std::cout<<"rvecs"<<std::endl;
                                std::cout<< rvecs << std::endl;
                                std::cout<<"tvecs"<<std::endl;
                                std::cout<<tvecs<<std::endl;

                                std::vector<cv::Point2f> outSideImgPoints;
                                cv::projectPoints(outSidePoints, rvecs, tvecs, cameraMatrix, distCoeffs, outSideImgPoints);
                                cv::Scalar color(255,0,255);
                                for(int i=0; i<outSideImgPoints.size(); i++){
                                        cv::circle(frame, outSideImgPoints[i], 7, color, 3);
                                }
                                cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs, tvecs, 2);
                                // std::cout<< "Corners: "<< cornerSet.size() << std::endl;
                                // std::cout<< "First corner: "<< cornerSet[0].x << "," << cornerSet[0].y << std::endl;
                                renderObject(frame, rvecs, tvecs, cameraMatrix, distCoeffs);
                        }
                        cv::drawChessboardCorners(frame, patternSize, cornerSet, patternFound);
                }else if(mode == 1){
                        std::vector<int> markerIds;
                        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
                        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
                        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
                        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
                        detector.detectMarkers(gray, markerCorners, markerIds, rejectedCandidates);
                        if(markerIds.size() == 1){
                                cv::solvePnP(arucoPointSet, markerCorners[0], cameraMatrix, distCoeffs, rvecs, tvecs);
                                cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs, tvecs, 2);
                                renderObject(frame, rvecs, tvecs, cameraMatrix, distCoeffs);
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
                        std::string fileName = "./captured/objProjection_" + capturedStr + ".jpg";
                        cv::imwrite(fileName,frame);
                }else if(key == 'g'){
                        cv::Mat markerImage;
                        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
                        cv::aruco::generateImageMarker(dictionary, 27, 200, markerImage, 1);
                        cv::imwrite("./captured/marker27.png", markerImage);
                }else if(key == 'm'){
                        mode == 1 ? mode = 0 : mode = 1;
                }
        }

        delete capdev;
        return(0);
}