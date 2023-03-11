#include <iostream>
#include <string>
#include <filesystem>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types_c.h>

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

        cv::Size patternSize(9,6);
        std::vector<cv::Vec3f> pointSet;
        generatePointSet(pointSet, patternSize);
        std::vector<std::vector<cv::Vec3f>> pointList;
        bool haveRecent = false;
        std::vector<cv::Point2f> recentCornerSet;
        cv::Mat recentShot;
        std::vector<std::vector<cv::Point2f>> cornerList;

        for(;;) {
                *capdev >> frame; // get a new frame from the camera, treat as a stream
                if( frame.empty() ) {
                  printf("frame is empty\n");
                  break;
                }
                
                cv::Mat gray;
                grayscale(frame, gray);
                
                std::vector<cv::Point2f> cornerSet;
                bool patternFound = cv::findChessboardCorners(gray, patternSize, cornerSet, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE+ cv::CALIB_CB_FAST_CHECK);

                if(patternFound){
                        cv::cornerSubPix(gray, cornerSet, cv::Size(11, 11), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                        cv::drawChessboardCorners(frame, patternSize, cornerSet, patternFound);
                        recentCornerSet = cornerSet;
                        recentShot = frame;
                        haveRecent = true;
                        // std::cout<< "Corners: "<< cornerSet.size() << std::endl;
                        // std::cout<< "First corner: "<< cornerSet[0].x << "," << cornerSet[0].y << std::endl;
                }

                cv::imshow("Video", frame);

                // see if there is a waiting keystroke
                char key = cv::pollKey();
                if( key == 'q') {
                    break;
                }else if (key == 's' && haveRecent){
                        pointList.push_back(pointSet);
                        cornerList.push_back(recentCornerSet);
                        captured += 1;
                        std::string capturedStr = std::to_string(captured);
                        std::string fileName = "./captured/capture_" + capturedStr + ".jpg";
                        cv::imwrite(fileName,recentShot);

                        if(cornerList.size()>=5){
                                calibrateAndSave(pointList, cornerList, refS);
                        }
                }
        }

        delete capdev;
        return(0);
}