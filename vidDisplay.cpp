#include <iostream>
#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>


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

        for(;;) {
                *capdev >> frame; // get a new frame from the camera, treat as a stream
                if( frame.empty() ) {
                  printf("frame is empty\n");
                  break;
                }
                
                cv::imshow("Video", frame);

                // see if there is a waiting keystroke
                char key = cv::waitKey(10);
                if( key == 'q') {
                    break;
                }else if (key == 's'){
                        captured += 1;
                        std::string capturedStr = std::to_string(captured);
                        std::string fileName = "./captured/capture_" + capturedStr + ".jpg";
                        std::cout << fileName << std::endl;
                        cv::imwrite(fileName,frame);
                }
        }

        delete capdev;
        return(0);
}