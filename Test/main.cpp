#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define IMAGE_NUM	(2)					/* 画像数 */
#define PAT_ROW		(7)					/* パターンの行数 */
#define PAT_COL		(10)				 /* パターンの列数 */
#define PAT_SIZE	 (PAT_ROW*PAT_COL)
#define ALL_POINTS (IMAGE_NUM*PAT_SIZE)
#define CHESS_SIZE (24.0)			 /* パターン1マスの1辺サイズ[mm] */

int main (int argc, char *argv[]) {
	cv::Mat src_img[IMAGE_NUM];
	CvSize pattern_size = cv::Size(PAT_COL, PAT_ROW);
	std::vector<std::vector<cv::Point3f> > all_object_points;
	cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distortion = cv::Mat::zeros(1, 8, CV_64F);

	// (1)キャリブレーション画像の読み込み
	for (int i = 0; i < IMAGE_NUM; i++) {
		char buf[32];
		sprintf(buf, "calib_img/%02d.jpg", i);
		src_img[i] = cv::imread(buf, CV_LOAD_IMAGE_COLOR);
	}

	// (2)3次元空間座標の設定
	for (int i = 0; i < IMAGE_NUM; i++) {
		std::vector<cv::Point3f> object_points;
		for (int j = 0; j < PAT_ROW; j++) {
			for (int k = 0; k < PAT_COL; k++) {
				object_points.push_back(cv::Point3f(j * CHESS_SIZE, k * CHESS_SIZE, 0.0f));
			}
		}
		all_object_points.push_back(object_points);
	}

	// (3)チェスボード（キャリブレーションパターン）のコーナー検出
	int found_num = 0;
	cvNamedWindow("Calibration", CV_WINDOW_AUTOSIZE);
	std::vector<std::vector<cv::Point2f> > all_corners;
	for (int i = 0; i < IMAGE_NUM; i++) {
		int corner_count;

		fprintf(stderr, "%02d...", i);

		std::vector<cv::Point2f> corners;
		if (cv::findChessboardCorners(src_img[i], pattern_size, corners)) {
		//int found = cvFindChessboardCorners(src_img[i], pattern_size, &corners[i * PAT_SIZE], &corner_count);
		//if (found) {
			fprintf (stderr, "ok\n");
			found_num++;
		} else {
			fprintf (stderr, "fail\n");
		}

		// (4)コーナー位置をサブピクセル精度に修正，描画
		cv::Mat src_gray(src_img[i].size(), CV_8UC1);
		cv::cvtColor(src_img[i], src_gray, CV_BGR2GRAY);
		//cvCvtColor(src_img[i], src_gray, CV_BGR2GRAY);
		cv::cornerSubPix(src_gray, corners, cv::Size(3, 3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

		all_corners.push_back(corners);

		cv::drawChessboardCorners(src_img[i], pattern_size, corners, true);
		cv::imshow("Calibration", src_img[i]);
		cv::waitKey(0);
	}
	cvDestroyWindow("Calibration");

	if (found_num != IMAGE_NUM)	return -1;

	// (5)内部パラメータ，歪み係数の推定
	std::vector<cv::Mat> rvecs, tvecs;
	cv::calibrateCamera(all_object_points, all_corners, src_img[0].size(), intrinsic, distortion, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6);

	printf("Intrinsic parameter:\n");
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c) {
			printf("%lf\t", intrinsic.at<double>(r, c));
		}
		printf("\n");
	}
	printf("Distortion:\n");
	for (int r = 0; r < distortion.rows; ++r) {
		for (int c = 0; c < distortion.cols; ++c) {
			printf("%lf\t", distortion.at<double>(r, c));
		}
		printf("\n");
	}
	for (int i = 0; i < rvecs.size(); ++i) {
		printf("Rotation %d:\n", i);
		for (int r = 0; r < rvecs[i].rows; ++r) {
			for (int c = 0; c < rvecs[i].cols; ++c) {
				printf("%.3lf\t", rvecs[i].at<double>(r, c));
			}
			printf("\n");
		}
		printf("\n");
	}
	for (int i = 0; i < tvecs.size(); ++i) {
	printf("Translation %d:\n", i);
		for (int r = 0; r < tvecs[i].rows; ++r) {
			for (int c = 0; c < tvecs[i].cols; ++c) {
				printf("%.3lf\t", tvecs[i].at<double>(r, c));
			}
			printf("\n");
		}
		printf("\n");
	}

	return 0;
}

