#ifndef HELPERS
#define HELPERS

int generatePointSet(std::vector<cv::Vec3f> &pointSet, cv::Size patternSize);

int calibrateAndSave(std::vector<std::vector<cv::Vec3f>> &pointList, std::vector<std::vector<cv::Point2f>> &cornerList, cv::Size imgSize);

void printDoubleMatrix(cv::Mat &matrix);

void saveIntrinsic(cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

#endif