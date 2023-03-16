#ifndef HELPERS
#define HELPERS

int grayscale( cv::Mat &src, cv::Mat &dst );

int generatePointSet(std::vector<cv::Vec3f> &pointSet, cv::Size patternSize);

int generateOutSidePoints(std::vector<cv::Vec3f> &outSidePoints, cv::Size patternSize);

void generateArucoPoints(std::vector<cv::Vec3f> &arucoPointSet);

int calibrateAndSave(std::vector<std::vector<cv::Vec3f>> &pointList, std::vector<std::vector<cv::Point2f>> &cornerList, cv::Size imgSize);

void printDoubleMatrix(cv::Mat &matrix);

void saveIntrinsic(cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

void renderObject(cv::Mat &frame, cv::Point2f position, cv::Mat &rvecs, cv::Mat &tvecs, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

void drawHarrisCorner(cv::Mat &frame, cv::Mat &corners);

#endif