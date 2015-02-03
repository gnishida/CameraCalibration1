#include "Calibration.h"


double Calibration::calibrateCamera(std::vector<std::vector<cv::Point3f> >& objectPoints, std::vector<std::vector<cv::Point2f> >& imagePoints, cv::Size size, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) {
	cv::calibrateCamera(objectPoints, imagePoints, size, cameraMat, distortion, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6);
	printf("Camera Matrix:\n");
	for (int r = 0; r < cameraMat.rows; ++r) {
		for (int c = 0; c < cameraMat.cols; ++c) {
			printf("%.3lf\t", cameraMat.at<double>(r, c));
		}
		printf("\n");
	}
	printf("\n");
	

	// 画像の数
	int n = objectPoints.size();

	rvecs.resize(n);
	tvecs.resize(n);

	std::vector<cv::Mat> H(n);
	for (int i = 0; i < n; ++i) {
		computeH(objectPoints[i], imagePoints[i], H[i]);
		printf("Homography Matrix:\n");
		for (int r = 0; r < H[i].rows; ++r) {
			for (int c = 0; c < H[i].cols; ++c) {
				printf("%.3lf\t", H[i].at<double>(r, c));
			}
			printf("\n");
		}
		printf("\n");
	}

	cv::Mat B;
	computeB(H, size, B);

	printf("B:\n");
	for (int r = 0; r < B.rows; ++r) {
		for (int c = 0; c < B.cols; ++c) {
			printf("%.3lf\t", B.at<double>(r, c));
		}
		printf("\n");
	}
	printf("\n");

	computeIntrinsicMatrix(B, cameraMat);
	printf("Camera Matrix:\n");
	for (int r = 0; r < cameraMat.rows; ++r) {
		for (int c = 0; c < cameraMat.cols; ++c) {
			printf("%.3lf\t", cameraMat.at<double>(r, c));
		}
		printf("\n");
	}
	printf("\n");

	for (int i = 0; i < n; ++i) {
		computeExtrinsicMatrix(cameraMat, H[i], rvecs[i], tvecs[i]);
	}

	double totalError = refine(objectPoints, imagePoints, cameraMat, distortion, rvecs, tvecs);

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
	if (rvec.cols == 3) {
		rvec.copyTo(R);
	} else {
		cv::Rodrigues(rvec, R);
	}

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

void Calibration::computeH(std::vector<cv::Point3f>& objectPoints, std::vector<cv::Point2f>& imagePoints, cv::Mat& H) {
	cv::Mat A(objectPoints.size() * 2, 9, CV_64F);
	for (int i = 0; i < objectPoints.size(); ++i) {
		A.at<double>(i * 2 + 0, 0) = objectPoints[i].x;
		A.at<double>(i * 2 + 0, 1) = objectPoints[i].y;
		A.at<double>(i * 2 + 0, 2) = 1;
		A.at<double>(i * 2 + 0, 3) = 0;
		A.at<double>(i * 2 + 0, 4) = 0;
		A.at<double>(i * 2 + 0, 5) = 0;
		A.at<double>(i * 2 + 0, 6) = -imagePoints[i].x * objectPoints[i].x;
		A.at<double>(i * 2 + 0, 7) = -imagePoints[i].x * objectPoints[i].y;
		A.at<double>(i * 2 + 0, 8) = -imagePoints[i].x;

		A.at<double>(i * 2 + 1, 0) = 0;
		A.at<double>(i * 2 + 1, 1) = 0;
		A.at<double>(i * 2 + 1, 2) = 0;
		A.at<double>(i * 2 + 1, 3) = objectPoints[i].x;
		A.at<double>(i * 2 + 1, 4) = objectPoints[i].y;
		A.at<double>(i * 2 + 1, 5) = 1;
		A.at<double>(i * 2 + 1, 6) = -imagePoints[i].y * objectPoints[i].x;
		A.at<double>(i * 2 + 1, 7) = -imagePoints[i].y * objectPoints[i].y;
		A.at<double>(i * 2 + 1, 8) = -imagePoints[i].y;
	}

	FILE* fp = fopen("test.txt", "w");
	fprintf(fp, "[");
	for (int r = 0; r < A.rows; ++r) {
		if (r > 0) fprintf(fp, ";\n");
		for (int c = 0; c < 9; ++c) {
			if (c > 0) fprintf(fp, ",");
			fprintf(fp, "%.3f", A.at<double>(r, c));
		}
	}
	fprintf(fp, "]\n\n");
	fclose(fp);

	cv::Mat w, u, v;
	cv::SVD::compute(A, w, u, v);

	/*
	for (int r = 0; r < v.rows; ++r) {
		for (int c = 0; c < v.cols; ++c) {
			printf("%.3f, ", v.at<double>(r, c));
		}
		printf("\n");
	}
	printf("\n");
	*/
	
	H = cv::Mat(3, 3, CV_64F);
	H.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
	H.at<double>(0, 1) = v.at<double>(v.rows - 1, 1);
	H.at<double>(0, 2) = v.at<double>(v.rows - 1, 2);
	H.at<double>(1, 0) = v.at<double>(v.rows - 1, 3);
	H.at<double>(1, 1) = v.at<double>(v.rows - 1, 4);
	H.at<double>(1, 2) = v.at<double>(v.rows - 1, 5);
	H.at<double>(2, 0) = v.at<double>(v.rows - 1, 6);
	H.at<double>(2, 1) = v.at<double>(v.rows - 1, 7);
	H.at<double>(2, 2) = v.at<double>(v.rows - 1, 8);


	// チェック
	/*
	for (int i = 0; i < objectPoints.size(); ++i) {
		cv::Mat pt(3, 1, CV_64F);
		pt.at<double>(0, 0) = objectPoints[i].x;
		pt.at<double>(1, 0) = objectPoints[i].y;
		pt.at<double>(2, 0) = 1;

		cv::Mat pt2 = H * pt;

		printf("(%.2lf, %.2lf) <-> (%.2lf, %.2lf)\n", imagePoints[i].x, imagePoints[i].y, pt2.at<double>(0, 0) / pt2.at<double>(2, 0), pt2.at<double>(1, 0) / pt2.at<double>(2, 0));
	}
	*/
}

void Calibration::computeB(std::vector<cv::Mat>& H, cv::Size& size, cv::Mat& B) {
	int n = H.size();

	double u0 = size.width * 0.5;
	double v0 = size.height * 0.5;

	cv::Mat A(n * 2, 3, CV_64F);
	for (int i = 0; i < n; ++i) {
		A.at<double>(i * 2 + 0, 0) = H[i].at<double>(0, 0) * H[i].at<double>(1, 0) - u0 * (H[i].at<double>(0, 0) * H[i].at<double>(1, 2) + H[i].at<double>(0, 2) * H[i].at<double>(1, 0));
		A.at<double>(i * 2 + 0, 1) = H[i].at<double>(0, 1) * H[i].at<double>(1, 1) - v0 * (H[i].at<double>(0, 1) * H[i].at<double>(1, 2) + H[i].at<double>(0, 2) * H[i].at<double>(1, 1));
		A.at<double>(i * 2 + 0, 2) = H[i].at<double>(0, 2) * H[i].at<double>(1, 2);

		A.at<double>(i * 2 + 1, 0) = H[i].at<double>(0, 0) * H[i].at<double>(0, 0) - H[i].at<double>(1, 0) * H[i].at<double>(1, 0) - 2 * u0 * (H[i].at<double>(0, 0) * H[i].at<double>(0, 2) - H[i].at<double>(1, 0) * H[i].at<double>(1, 2));
		A.at<double>(i * 2 + 1, 1) = H[i].at<double>(0, 1) * H[i].at<double>(0, 1) - H[i].at<double>(1, 1) * H[i].at<double>(1, 1) - 2 * v0 * (H[i].at<double>(0, 1) * H[i].at<double>(0, 2) - H[i].at<double>(1, 1) * H[i].at<double>(1, 2));
		A.at<double>(i * 2 + 1, 2) = H[i].at<double>(0, 2) * H[i].at<double>(0, 2) - H[i].at<double>(1, 2) * H[i].at<double>(1, 2);
	}

	cv::Mat w, u, v;
	cv::SVD::compute(A, w, u, v);

	B = cv::Mat(3, 3, CV_64F);
	B.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
	B.at<double>(0, 1) = 0;
	B.at<double>(0, 2) = -u0 * v.at<double>(v.rows - 1, 0);
	B.at<double>(1, 0) = 0;
	B.at<double>(1, 1) = v.at<double>(v.rows - 1, 1);
	B.at<double>(1, 2) = -v0 * v.at<double>(v.rows - 1, 1);
	B.at<double>(2, 0) = -u0 * v.at<double>(v.rows - 1, 0);
	B.at<double>(2, 1) = -v0 * v.at<double>(v.rows - 1, 1);
	B.at<double>(2, 2) = v.at<double>(v.rows - 1, 2);
}

void Calibration::computeIntrinsicMatrix(cv::Mat& B, cv::Mat& cameraMat) {
	double v0 = (B.at<double>(0, 1) * B.at<double>(0, 2) - B.at<double>(0, 0) * B.at<double>(1, 2)) / (B.at<double>(0, 0) * B.at<double>(1, 1) - B.at<double>(0, 1) * B.at<double>(0, 1));
	double lmbd = B.at<double>(2, 2) - (B.at<double>(0, 2) * B.at<double>(0, 2) + v0 * (B.at<double>(0, 1) * B.at<double>(0, 2) - B.at<double>(0, 0) * B.at<double>(1, 2))) / B.at<double>(0, 0);
	double alpha = sqrt(lmbd / B.at<double>(0, 0));
	double beta = sqrt(lmbd * B.at<double>(0, 0) / (B.at<double>(0, 0) * B.at<double>(1, 1) - B.at<double>(0, 1) * B.at<double>(0, 1)));
	double gamma = -B.at<double>(0, 1) * alpha * alpha * beta / lmbd;
	double u0 = gamma * v0 / beta - B.at<double>(0, 2) * alpha * alpha / lmbd;

	cameraMat.at<double>(0, 0) = alpha;
	cameraMat.at<double>(0, 1) = gamma;
	cameraMat.at<double>(0, 2) = u0;
	cameraMat.at<double>(1, 0) = 0;
	cameraMat.at<double>(1, 1) = beta;
	cameraMat.at<double>(1, 2) = v0;
	cameraMat.at<double>(2, 0) = 0;
	cameraMat.at<double>(2, 1) = 0;
	cameraMat.at<double>(2, 2) = 1;
}

void Calibration::computeExtrinsicMatrix(cv::Mat& cameraMat, cv::Mat& H, cv::Mat& R, cv::Mat& T) {
	R = cameraMat.inv() * H;
	T = cv::Mat(3, 1, CV_64F);
	for (int r = 0; r < 3; ++r) {
		T.at<double>(r, 0) = R.at<double>(r, 2);
	}

	cv::Mat r1(R, cv::Rect(0, 0, 1, 3));
	cv::Mat r2(R, cv::Rect(1, 0, 1, 3));
	cv::Mat r3(R, cv::Rect(2, 0, 1, 3));
	r3 = r1.cross(r2);
}

double Calibration::refine(std::vector<std::vector<cv::Point3f> >& objectPoints, std::vector<std::vector<cv::Point2f> >& imagePoints, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) {
	return 0.0;
}
