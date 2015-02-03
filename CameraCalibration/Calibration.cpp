#include "Calibration.h"

#define SQR(x)	((x) * (x))

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
	for (int i = 0; i < rvecs.size(); ++i) {
		printf("R:\n");
		cv::Mat dst;
		//rodrigues(rvecs[i], dst);
		cv::Rodrigues(rvecs[i], dst);
		for (int r = 0; r < dst.rows; ++r) {
			for (int c = 0; c < dst.cols; ++c) {
				printf("%.3lf\t", dst.at<double>(r, c));
			}
			printf("\n");
		}
		printf("\n");

		printf("T:\n");
		for (int r = 0; r < tvecs[i].rows; ++r) {
			for (int c = 0; c < tvecs[i].cols; ++c) {
				printf("%.3lf\t", tvecs[i].at<double>(r, c));
			}
			printf("\n");
		}
		printf("\n");
	}
	

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

	computeIntrinsicMatrix(B, cameraMat);
	printf("Camera Matrix:\n");
	for (int r = 0; r < cameraMat.rows; ++r) {
		for (int c = 0; c < cameraMat.cols; ++c) {
			printf("%.3lf\t", cameraMat.at<double>(r, c));
		}
		printf("\n");
	}
	printf("\n");

	cameraMat.at<double>(0, 0) = 780;
	cameraMat.at<double>(1, 1) = 780;

	for (int i = 0; i < n; ++i) {
		computeExtrinsicMatrix(cameraMat, H[i], rvecs[i], tvecs[i]);

		printf("R:\n");
		for (int r = 0; r < rvecs[i].rows; ++r) {
			for (int c = 0; c < rvecs[i].cols; ++c) {
				printf("%.3lf\t", rvecs[i].at<double>(r, c));
			}
			printf("\n");
		}
		printf("\n");

		printf("T:\n");
		for (int r = 0; r < tvecs[i].rows; ++r) {
			for (int c = 0; c < tvecs[i].cols; ++c) {
				printf("%.3lf\t", tvecs[i].at<double>(r, c));
			}
			printf("\n");
		}
		printf("\n");
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
void Calibration::projectPoints(std::vector<cv::Point3f> objectPoints, cv::Mat& rvec, cv::Mat& tvec, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Point2f>& projectedImagePoints) {
	// 外部パラメータ行列の作成
	cv::Mat R;
	//rodrigues(rvec, R);
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

void Calibration::rodrigues(cv::Mat& src, cv::Mat& dst) {
	if (src.cols == 1) {
		double theta = cv::norm(src);
		src /= theta;

		dst = cv::Mat::zeros(3, 3, CV_64F);
		dst.at<double>(0, 1) = -src.at<double>(2, 0);
		dst.at<double>(0, 2) = src.at<double>(1, 0);
		dst.at<double>(1, 0) = src.at<double>(2, 0);
		dst.at<double>(1, 2) = -src.at<double>(0, 0);
		dst.at<double>(2, 0) = -src.at<double>(1, 0);
		dst.at<double>(2, 1) = src.at<double>(0, 0);

		dst = dst * sin(theta) + cv::Mat::eye(3, 3, CV_64F) * cos(theta) + src * src.t() * (1 - cos(theta));
		cv::Rodrigues(src, dst);
	} else {
		dst = cv::Mat(3, 1, CV_64F);
		cv::Mat tmp = (src - src.t()) * 0.5;
		dst.at<double>(0, 0) = tmp.at<double>(2, 1);
		dst.at<double>(1, 0) = tmp.at<double>(0, 2);
		dst.at<double>(2, 0) = tmp.at<double>(1, 0);
		double theta = cv::norm(dst);
		dst /= theta;
		printf("Rodg\n");
		for (int r = 0; r < dst.rows; ++r) {
			printf("%.3lf, ", dst.at<double>(r, 0));
		}
		printf("\n");

		cv::Rodrigues(src, dst);
		printf("Rodg\n");
		for (int r = 0; r < dst.rows; ++r) {
			printf("%.3lf, ", dst.at<double>(r, 0));
		}
		printf("\n");
	}
}

/**
 * Homogeneous 行列を計算する。Zhang論文の式(2)を参照のこと。
 *
 * @param objectPoints		3D座標のリスト
 * @param imagePoints		2D座標のリスト
 * @param H					計算されたHomogeneous行列
 */
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

	cv::Mat w, u, v;
	cv::SVD::compute(A, w, u, v);
	
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

/**
 * 行列B=(AA^T)^-1を計算する。Zhang論文の式(9)を参照のこと。
 * 
 * @param H			homogeneous行列
 * @param size		画像のサイズ
 * @param B			計算された行列B
 */
void Calibration::computeB(std::vector<cv::Mat>& H, cv::Size& size, cv::Mat& B) {
	int n = H.size();

	cv::Mat A(n * 2 + 3, 6, CV_64F);
	for (int i = 0; i < n; ++i) {
		double h11 = H[i].at<double>(0, 0);
		double h12 = H[i].at<double>(0, 1);
		double h13 = H[i].at<double>(0, 2);
		double h21 = H[i].at<double>(1, 0);
		double h22 = H[i].at<double>(1, 1);
		double h23 = H[i].at<double>(1, 2);
		double h31 = H[i].at<double>(2, 0);
		double h32 = H[i].at<double>(2, 1);
		double h33 = H[i].at<double>(2, 2);

		A.at<double>(i * 2 + 0, 0) = h11 * h11;
		A.at<double>(i * 2 + 0, 1) = h11 * h22 + h12 * h21;
		A.at<double>(i * 2 + 0, 2) = h12 * h22;
		A.at<double>(i * 2 + 0, 3) = h11 * h23 + h13 * h21;
		A.at<double>(i * 2 + 0, 4) = h12 * h23 + h13 * h22;
		A.at<double>(i * 2 + 0, 5) = h13 * h23;

		A.at<double>(i * 2 + 1, 0) = h11 * h11 - h21 * h21;
		A.at<double>(i * 2 + 1, 1) = h11 * h12 * 2 - h21 * h22 * 2;
		A.at<double>(i * 2 + 1, 2) = h11 * h12 - h22 * h22;
		A.at<double>(i * 2 + 1, 3) = h11 * h13 * 2 - h21 * h23 * 2;
		A.at<double>(i * 2 + 1, 4) = h12 * h13 * 2 - h22 * h23 * 2;
		A.at<double>(i * 2 + 1, 5) = h13 * h13 - h23 * h23;
	}
	{
		A.at<double>(n * 2 + 0, 0) = 0;
		A.at<double>(n * 2 + 0, 1) = 1;
		A.at<double>(n * 2 + 0, 2) = 0;
		A.at<double>(n * 2 + 0, 3) = 0;
		A.at<double>(n * 2 + 0, 4) = 0;
		A.at<double>(n * 2 + 0, 5) = 0;

		A.at<double>(n * 2 + 1, 0) = size.width * 0.5;
		A.at<double>(n * 2 + 1, 1) = 0;
		A.at<double>(n * 2 + 1, 2) = 0;
		A.at<double>(n * 2 + 1, 3) = 1;
		A.at<double>(n * 2 + 1, 4) = 0;
		A.at<double>(n * 2 + 1, 5) = 0;

		A.at<double>(n * 2 + 2, 0) = 0;
		A.at<double>(n * 2 + 2, 1) = 0;
		A.at<double>(n * 2 + 2, 2) = size.height * 0.5;
		A.at<double>(n * 2 + 2, 3) = 0;
		A.at<double>(n * 2 + 2, 4) = 1;
		A.at<double>(n * 2 + 2, 5) = 0;
	}

	cv::Mat w, u, v;
	cv::SVD::compute(A, w, u, v);

	double B11 = v.at<double>(v.rows - 1, 0);
	double B12 = v.at<double>(v.rows - 1, 1);
	double B22 = v.at<double>(v.rows - 1, 2);
	double B13 = v.at<double>(v.rows - 1, 3);
	double B23 = v.at<double>(v.rows - 1, 4);
	double B33 = v.at<double>(v.rows - 1, 5);

	B = cv::Mat(3, 3, CV_64F);
	B.at<double>(0, 0) = B11;
	B.at<double>(0, 1) = B12;
	B.at<double>(0, 2) = B13;
	B.at<double>(1, 0) = B12;
	B.at<double>(1, 1) = B22;
	B.at<double>(1, 2) = B23;
	B.at<double>(2, 0) = B13;
	B.at<double>(2, 1) = B23;
	B.at<double>(2, 2) = B33;

	if (B11 < 0) {
		B = -B;
	}
}

void Calibration::computeIntrinsicMatrix(cv::Mat& B, cv::Mat& cameraMat) {
	double v0 = (B.at<double>(0, 1) * B.at<double>(0, 2) - B.at<double>(0, 0) * B.at<double>(1, 2)) / (B.at<double>(0, 0) * B.at<double>(1, 1) - B.at<double>(0, 1) * B.at<double>(0, 1));
	double lmbd = B.at<double>(2, 2) - (B.at<double>(0, 2) * B.at<double>(0, 2) + v0 * (B.at<double>(0, 1) * B.at<double>(0, 2) - B.at<double>(0, 0) * B.at<double>(1, 2))) / B.at<double>(0, 0);
	if (lmbd < 0) lmbd = -lmbd;
	double alpha = sqrt(lmbd / B.at<double>(0, 0));
	double beta = sqrt(lmbd * B.at<double>(0, 0) / (B.at<double>(0, 0) * B.at<double>(1, 1) - B.at<double>(0, 1) * B.at<double>(0, 1)));
	double gamma = -B.at<double>(0, 1) * alpha * alpha * beta / lmbd;
	double u0 = gamma * v0 / alpha - B.at<double>(0, 2) * alpha * alpha / lmbd;

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

/**
 * カメラ外部パラメータ行列を計算する
 *
 * @param cameraMat		カメラ内部パラメータ行列
 * @param H				homogeneous行列
 * @param R				計算された回転行列(3x1)
 * @param T				計算された並進行列(3x1)
 */
void Calibration::computeExtrinsicMatrix(cv::Mat& cameraMat, cv::Mat& H, cv::Mat& R, cv::Mat& T) {
	cv::Mat R33(3, 3, CV_64F);
	R33 = cameraMat.inv() * H;
	double lmbd = -1.0 / sqrt(R33.at<double>(0, 0) * R33.at<double>(0, 0) + R33.at<double>(1, 0) * R33.at<double>(1, 0) + R33.at<double>(2, 0) * R33.at<double>(2, 0));

	R33 = R33 * lmbd;

	T = cv::Mat(3, 1, CV_64F);
	for (int r = 0; r < 3; ++r) {
		T.at<double>(r, 0) = R33.at<double>(r, 2);
	}

	// cross productにより回転行列の３列目を計算
	{
		R33.at<double>(0, 2) = R33.at<double>(1, 0) * R33.at<double>(2, 1) - R33.at<double>(2, 0) * R33.at<double>(1, 1);
		R33.at<double>(1, 2) = R33.at<double>(2, 0) * R33.at<double>(0, 1) - R33.at<double>(0, 0) * R33.at<double>(2, 1);
		R33.at<double>(2, 2) = R33.at<double>(0, 0) * R33.at<double>(1, 1) - R33.at<double>(1, 0) * R33.at<double>(0, 1);
	}

	// 回転行列の各列をnormalize
	for (int c = 0; c < 3; ++c) {
		double l = sqrt(R33.at<double>(0, c) * R33.at<double>(0, c) + R33.at<double>(1, c) * R33.at<double>(1, c) + R33.at<double>(2, c) * R33.at<double>(2, c));
		for (int r = 0; r < 3; ++r) {
			R33.at<double>(r, c) /= l;
		}
	}

	// 3x1行列に変換する
	//rodrigues(R33, R);
	cv::Rodrigues(R33, R);
}

double Calibration::refine(std::vector<std::vector<cv::Point3f> >& objectPoints, std::vector<std::vector<cv::Point2f> >& imagePoints, cv::Mat& cameraMat, cv::Mat& distortion, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) {
	// パラメータの数
	const int NUM_PARAMS = 18;

	// 画像数
	int n = objectPoints.size();

	// データ数
	int total_m = 0;
	for (int i = 0; i < n; ++i) {
		total_m += objectPoints[i].size();
	}

	// パラメータ（alpha, beta, gamma, u0, v0, (rx, ry, rz, tx, ty, tz) * 画像数 )
	real x[NUM_PARAMS];

	// パラメータの初期推定値
	x[0] = cameraMat.at<double>(0, 0); // alpha
	x[1] = cameraMat.at<double>(1, 1); // beta
	x[2] = cameraMat.at<double>(0, 1); // gamma
	x[3] = cameraMat.at<double>(0, 2); // u0
	x[4] = cameraMat.at<double>(1, 2); // v0
	x[5] = 0; // distortion
	for (int i = 0; i < rvecs.size(); ++i) {
		x[i * 6 + 6] = rvecs[i].at<double>(0, 0); // rx
		x[i * 6 + 7] = rvecs[i].at<double>(1, 0); // ry
		x[i * 6 + 8] = rvecs[i].at<double>(2, 0); // rz
		x[i * 6 + 9] = tvecs[i].at<double>(0, 0); // tx
		x[i * 6 + 10] = tvecs[i].at<double>(1, 0); // ty
		x[i * 6 + 11] = tvecs[i].at<double>(2, 0); // tz
	}

	// 観測データ（total_m * (x,y)の２個分）
	real* y = new real[total_m * 2];
	int index = 0;
	for (int i = 0; i < imagePoints.size(); ++i) {
		for (int j = 0; j < imagePoints[i].size(); ++j) {
			y[index++] = imagePoints[i][j].x;
			y[index++] = imagePoints[i][j].y;
		}
	}

	// 真値と観測データとの誤差が格納される配列
	real* fvec = new real[total_m * 2];

	// 結果のヤコビ行列
	real* fjac = new real[total_m * 2 * NUM_PARAMS];

	// lmdif内部使用パラメータ
	int ipvt[NUM_PARAMS];

	real diag[NUM_PARAMS], qtf[NUM_PARAMS], wa1[NUM_PARAMS], wa2[NUM_PARAMS], wa3[NUM_PARAMS];
	real* wa4 = new real[total_m * 2];

	// 観測データを格納する構造体オブジェクト
	fcndata_t data;
	data.m = total_m * 2;
	data.y = y;
	data.objectPoints = &objectPoints;
	data.imagePoints = &imagePoints;

	// 観測データの数と同じ値にすることを推奨する
	int ldfjac = total_m * 2;

	// 各種パラメータ（推奨値のまま）
	real ftol = sqrt(__cminpack_func__(dpmpar)(1));
	real xtol = sqrt(__cminpack_func__(dpmpar)(1));
	real gtol = 0.;

	// 最大何回繰り返すか？
	int maxfev = 1600;

	// 収束チェック用の微小値
	real epsfcn = 1e-010;//1e-08;
	int mode = 1;

	// 1が推奨されている？
	real factor = 1;//1.e2;

	// 実際に繰り返した回数
	int nfev;

	int nprint = 0;
	int info = __cminpack_func__(lmdif)(fcn, &data, total_m * 2, NUM_PARAMS, x, fvec, ftol, xtol, gtol, maxfev, epsfcn,
									diag, mode, factor, nprint, &nfev, fjac, ldfjac, ipvt, qtf, wa1, wa2, wa3, wa4);
	real fnorm = __cminpack_func__(enorm)(total_m * 2, fvec);

	printf("final l2 norm of the residuals: %15.7g\n\n", (double)fnorm);
	printf("number of function evaluations: %10i\n\n", nfev);
	printf("exit parameter %10i\n\n", info);

	// 収束結果を反映する
	cameraMat.at<double>(0, 0) = x[0]; // alpha
	cameraMat.at<double>(1, 1) = x[1]; // beta
	cameraMat.at<double>(0, 1) = x[2]; // gamma
	cameraMat.at<double>(0, 2) = x[3]; // u0
	cameraMat.at<double>(1, 2) = x[4]; // v0
	distortion.at<double>(0, 0) = x[5]; // distortion
	for (int i = 0; i < rvecs.size(); ++i) {
		rvecs[i].at<double>(0, 0) = x[i * 6 + 6]; // rx
		rvecs[i].at<double>(1, 0) = x[i * 6 + 7]; // ry
		rvecs[i].at<double>(2, 0) = x[i * 6 + 8]; // rz
		tvecs[i].at<double>(0, 0) = x[i * 6 + 9]; // tx
		tvecs[i].at<double>(1, 0) = x[i * 6 + 10]; // ty
		tvecs[i].at<double>(2, 0) = x[i * 6 + 11]; // tz
	}

	// 真値と観測データの差を合計する
	double total_error = 0.0;
	for (int i = 0; i < 2; ++i) {
		std::vector<cv::Point2f> projectedImagePoints;
		
		projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMat, distortion, projectedImagePoints);

		for (int j = 0; j < 70; ++j) {
			// 射影結果と観測データの誤差
			total_error += sqrt(SQR(projectedImagePoints[j].x - imagePoints[i][j].x) + SQR(projectedImagePoints[j].y - imagePoints[i][j].y));
		}
	}

	// メモリ解放
	delete [] y;
	delete [] fvec;
	delete [] fjac;
	delete [] wa4;

	return total_error / 140;
}

/**
 * 自分の関数を記述し、真値と観測データとの差を計算する。
 *
 * @param p		観測データが入った構造体オブジェクト
 * @param m		観測データの数
 * @param n		パラメータの数
 * @param x		パラメータ配列
 * @param fvec	真値と観測データとの差を格納する配列
 * @param iflag	lmdifから返されるフラグ (0なら終了?)
 * @return		0を返却する
 */
int Calibration::fcn(void *p, int m, int n, const real *x, real *fvec, int iflag) {
	const real *y = ((fcndata_t*)p)->y;

	if (iflag == 0) {
		/* insert print statements here when nprint is positive. */
		/* if the nprint parameter to lmdif is positive, the function is
		called every nprint iterations with iflag=0, so that the
		function may perform special operations, such as printing
		residuals. */
		return 0;
	}

	// 画像の数
	int N = ((fcndata_t*)p)->objectPoints->size();

	// カメラ内部パラメータ行列
	cv::Mat cameraMat = cv::Mat::eye(3, 3, CV_64F);
	cameraMat.at<double>(0, 0) = x[0]; // alpha
	cameraMat.at<double>(1, 1) = x[1]; // beta
	cameraMat.at<double>(0, 1) = x[2]; // gamma
	cameraMat.at<double>(0, 2) = x[3]; // u0
	cameraMat.at<double>(1, 2) = x[4]; // v0

	// distortion
	cv::Mat distortion = cv::Mat::zeros(1, 8, CV_64F);
	distortion.at<double>(0, 0) = x[5]; // distortion

	std::vector<cv::Mat> rvecs(2);
	std::vector<cv::Mat> tvecs(2);
	for (int i = 0; i < N; ++i) {
		// カメラ外部パラメータ行列
		rvecs[i] = cv::Mat(3, 1, CV_64F);
		rvecs[i].at<double>(0, 0) = x[i * 6 + 6];
		rvecs[i].at<double>(1, 0) = x[i * 6 + 7];
		rvecs[i].at<double>(2, 0) = x[i * 6 + 8];

		tvecs[i] = cv::Mat(3, 1, CV_64F);
		tvecs[i].at<double>(0, 0) = x[i * 6 + 9];
		tvecs[i].at<double>(1, 0) = x[i * 6 + 10];
		tvecs[i].at<double>(2, 0) = x[i * 6 + 11];
	}

	// 真値と観測データの差を計算する
	int index = 0;
	for (int i = 0; i < N; ++i) {
		std::vector<cv::Point2f> projectedImagePoints;
		
		projectPoints(((fcndata_t*)p)->objectPoints->at(i), rvecs[i], tvecs[i], cameraMat, distortion, projectedImagePoints);

		for (int j = 0; j < ((fcndata_t*)p)->objectPoints->at(i).size(); ++j) {
			// 射影結果と観測データの誤差を格納する
			fvec[index++] = SQR(projectedImagePoints[j].x - ((fcndata_t*)p)->imagePoints->at(i).at(j).x);
			fvec[index++] = SQR(projectedImagePoints[j].y - ((fcndata_t*)p)->imagePoints->at(i).at(j).y);
		}
	}

	return 0;
}
