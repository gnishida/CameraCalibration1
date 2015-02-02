#include "Calibration.h"


double Calibration::calibrateCamera(std::vector<std::vector<cv::Point3f> >& objectPoints, std::vector<std::vector<cv::Point2f> >& imagePoints, cv::Size size, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) {
	double totalError = cv::calibrateCamera(objectPoints, imagePoints, size, cameraMat, distortion, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6);
	return totalError;
}

void Calibration::projectPoints(std::vector<cv::Point3f>& objectPoints, cv::Mat& rvec, cv::Mat& tvec, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Point2f>& projectedImagePoints) {
	cv::projectPoints(objectPoints, rvec, tvec, cameraMat, distortion, projectedImagePoints);
}
