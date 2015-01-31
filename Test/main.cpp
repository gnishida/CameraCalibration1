#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define IMAGE_NUM  (2)          /* 画像数 */
#define PAT_ROW    (7)          /* パターンの行数 */
#define PAT_COL    (10)         /* パターンの列数 */
#define PAT_SIZE   (PAT_ROW*PAT_COL)
#define ALL_POINTS (IMAGE_NUM*PAT_SIZE)
#define CHESS_SIZE (24.0)       /* パターン1マスの1辺サイズ[mm] */

int main (int argc, char *argv[]) {
  int i, j, k;
  int corner_count, found;
  int p_count[IMAGE_NUM];
  IplImage *src_img[IMAGE_NUM];
  CvSize pattern_size = cvSize (PAT_COL, PAT_ROW);
  CvPoint3D32f objects[ALL_POINTS];
  CvPoint2D32f *corners = (CvPoint2D32f *) cvAlloc (sizeof (CvPoint2D32f) * ALL_POINTS);
  CvMat object_points;
  CvMat image_points;
  CvMat point_counts;
  CvMat *intrinsic = cvCreateMat (3, 3, CV_32FC1);
  CvMat *rotation = cvCreateMat (1, 3, CV_32FC1);
  CvMat *translation = cvCreateMat (1, 3, CV_32FC1);
  CvMat *distortion = cvCreateMat (1, 4, CV_32FC1);

  // (1)キャリブレーション画像の読み込み
  for (i = 0; i < IMAGE_NUM; i++) {
    char buf[32];
    sprintf (buf, "calib_img/%02d.jpg", i);
    if ((src_img[i] = cvLoadImage (buf, CV_LOAD_IMAGE_COLOR)) == NULL) {
      fprintf (stderr, "cannot load image file : %s\n", buf);
    }
  }

  // (2)3次元空間座標の設定
  for (i = 0; i < IMAGE_NUM; i++) {
    for (j = 0; j < PAT_ROW; j++) {
      for (k = 0; k < PAT_COL; k++) {
        objects[i * PAT_SIZE + j * PAT_COL + k].x = j * CHESS_SIZE;
        objects[i * PAT_SIZE + j * PAT_COL + k].y = k * CHESS_SIZE;
        objects[i * PAT_SIZE + j * PAT_COL + k].z = 0.0;
      }
    }
  }
  cvInitMatHeader (&object_points, ALL_POINTS, 3, CV_32FC1, objects);

  // (3)チェスボード（キャリブレーションパターン）のコーナー検出
  int found_num = 0;
  cvNamedWindow ("Calibration", CV_WINDOW_AUTOSIZE);
  for (i = 0; i < IMAGE_NUM; i++) {
    found = cvFindChessboardCorners (src_img[i], pattern_size, &corners[i * PAT_SIZE], &corner_count);
    fprintf (stderr, "%02d...", i);
    if (found) {
      fprintf (stderr, "ok\n");
      found_num++;
    }
    else {
      fprintf (stderr, "fail\n");
    }
    // (4)コーナー位置をサブピクセル精度に修正，描画
    IplImage *src_gray = cvCreateImage (cvGetSize (src_img[i]), IPL_DEPTH_8U, 1);
    cvCvtColor (src_img[i], src_gray, CV_BGR2GRAY);
    cvFindCornerSubPix (src_gray, &corners[i * PAT_SIZE], corner_count,
                        cvSize (3, 3), cvSize (-1, -1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
    cvDrawChessboardCorners (src_img[i], pattern_size, &corners[i * PAT_SIZE], corner_count, found);
    p_count[i] = corner_count;
    cvShowImage ("Calibration", src_img[i]);
    cvWaitKey (0);
  }
  cvDestroyWindow ("Calibration");

  if (found_num != IMAGE_NUM)
    return -1;
  cvInitMatHeader (&image_points, ALL_POINTS, 1, CV_32FC2, corners);
  cvInitMatHeader (&point_counts, IMAGE_NUM, 1, CV_32SC1, p_count);

  // (5)内部パラメータ，歪み係数の推定
  cvCalibrateCamera2 (&object_points, &image_points, &point_counts, cvSize (640, 480), intrinsic, distortion);

  // (6)外部パラメータの推定
  CvMat sub_image_points, sub_object_points;
  int base = 0;
  cvGetRows (&image_points, &sub_image_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
  cvGetRows (&object_points, &sub_object_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
  cvFindExtrinsicCameraParams2 (&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);

  // (7)XMLファイルへの書き出し
  CvFileStorage *fs;
  fs = cvOpenFileStorage ("camera.xml", 0, CV_STORAGE_WRITE);
  cvWrite (fs, "intrinsic", intrinsic);
  cvWrite (fs, "rotation", rotation);
  cvWrite (fs, "translation", translation);
  cvWrite (fs, "distortion", distortion);
  cvReleaseFileStorage (&fs);

  for (i = 0; i < IMAGE_NUM; i++) {
    cvReleaseImage (&src_img[i]);
  }

  return 0;
}

