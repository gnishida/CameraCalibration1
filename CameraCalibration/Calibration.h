#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>

class Calibration {
protected:
	Calibration() {}

public:
	static double calibrateCamera(std::vector<std::vector<cv::Point3f> >& objectPoints, std::vector<std::vector<cv::Point2f> >& imagePoints, cv::Size size, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs);
	static void projectPoints(std::vector<cv::Point3f>& objectPoints, cv::Mat& rvec, cv::Mat& tvec, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Point2f>& projectedImagePoints);
};

