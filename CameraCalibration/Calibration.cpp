#include "Calibration.h"


double Calibration::calibrateCamera(std::vector<std::vector<cv::Point3f> >& objectPoints, std::vector<std::vector<cv::Point2f> >& imagePoints, cv::Size size, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) {
	double totalError = cv::calibrateCamera(objectPoints, imagePoints, size, cameraMat, distortion, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6);
	return totalError;
}

/**
 * Project 3D points onto the image plane based on the intrinsic and extrinsic parameters.
 *
 * @param objectPoints			3D points
 * @param rvec					rotation parameters
 * @param tvec					translation parameters
 * @param cameraMat				intrinsic parameters
 * @param distortion			distortion parameters
 * @param projectedImagePoints	the projected points on the image plane
 */
void Calibration::projectPoints(std::vector<cv::Point3f>& objectPoints, cv::Mat& rvec, cv::Mat& tvec, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Point2f>& projectedImagePoints) {
	// 外部パラメータ行列の作成
	cv::Mat R;
	cv::Rodrigues(rvec, R);

	cv::Mat P(3, 4, CV_64F);
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c) {
			P.at<double>(r, c) = R.at<double>(r, c);
		}
		P.at<double>(r, 3) = tvec.at<double>(r, 0);
	}

	// レンズ歪み
	double k1 = distortion.at<double>(0, 0);

	// principal points
	double u0 = cameraMat.at<double>(0, 2);
	double v0 = cameraMat.at<double>(1, 2);
	double fx = cameraMat.at<double>(0, 0);
	double fy = cameraMat.at<double>(1, 1);

	projectedImagePoints.resize(objectPoints.size());

	for (int i = 0; i < objectPoints.size(); ++i) {
		cv::Mat pts(4, 1, CV_64F);
		pts.at<double>(0, 0) = objectPoints[i].x;
		pts.at<double>(1, 0) = objectPoints[i].y;
		pts.at<double>(2, 0) = objectPoints[i].z;
		pts.at<double>(3, 0) = 1.0;

		cv::Mat pts2 = cameraMat * P * pts;

		double u = pts2.at<double>(0, 0) / pts2.at<double>(2, 0);
		double v = pts2.at<double>(1, 0) / pts2.at<double>(2, 0);
		double x = (u - u0) / fx;
		double y = (v - v0) / fy;

		double r2 = x * x + y * y;

		double du = (u - u0) * k1 * r2;
		double dv = (v - v0) * k1 * r2;

		projectedImagePoints[i].x = u + du;
		projectedImagePoints[i].y = v + dv;
	}
}
