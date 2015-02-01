#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags) : QMainWindow(parent, flags) {
	ui.setupUi(this);

	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionOpenImages, SIGNAL(triggered()), this, SLOT(openImages()));

	// setup the OpenGL widget
	glWidget = new GLWidget3D();
	//setCentralWidget(glWidget);

	QWidget* w = new QWidget(this);
	this->centralWidget()->setLayout(new QHBoxLayout());
	this->centralWidget()->layout()->addWidget(w);
	this->centralWidget()->layout()->addWidget(glWidget);

	w->setFixedSize(400, 500);
	w->setLayout(new QVBoxLayout);
	w->layout()->addWidget(&imgWidget[0]);
	w->layout()->addWidget(&imgWidget[1]);

	imgWidget[0].setFixedSize(400, 250);
	imgWidget[1].setFixedSize(400, 250);
}

MainWindow::~MainWindow() {
}

void MainWindow::openImages() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Image files (*.jpg)"));
	if (filename.isEmpty()) return;

	imgWidget[0].setImage(filename);

	filename = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Image files (*.jpg)"));
	if (filename.isEmpty()) return;

	imgWidget[1].setImage(filename);


	// カメラ内部パラメータ、歪み係数、外部パラメータの推定
	std::vector<std::vector<cv::Point3f> > objectPoints;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	for (int i = 0; i < 2; ++i) {
		objectPoints.push_back(imgWidget[i].objectPoints);
		imagePoints.push_back(imgWidget[i].imagePoints);
	}
	cv::Mat cameraMat = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distortion = cv::Mat::zeros(1, 8, CV_64F);
	std::vector<cv::Mat> rvecs, tvecs;
	double totalError = cv::calibrateCamera(objectPoints, imagePoints, imgWidget[0].imgMat.size(), cameraMat, distortion, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6);
	printf("Total error: %lf\n", totalError);

	glWidget->A = cv::Mat::eye(4, 4, CV_64F);
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c) {
			glWidget->A.at<double>(r, c) = cameraMat.at<double>(r, c);
		}
	}
	for (int i = 0; i < 2; ++i) {
		cv::Mat dst;
		cv::Rodrigues(rvecs[i], dst);

		glWidget->P[i] = cv::Mat::eye(4, 4, CV_64F);
		for (int r = 0; r < 3; ++r) {
			for (int c = 0; c < 3; ++c) {
				glWidget->P[i].at<double>(r, c) = dst.at<double>(r, c);
			}
		}
		for (int r = 0; r < 3; ++r) {
			glWidget->P[i].at<double>(r, 3) = tvecs[i].at<double>(r, 0);
		}
	}

	glWidget->initialized = true;
}
