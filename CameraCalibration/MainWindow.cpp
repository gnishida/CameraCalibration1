#include "MainWindow.h"
#include "Calibration.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags) : QMainWindow(parent, flags) {
	ui.setupUi(this);

	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionOpenImages, SIGNAL(triggered()), this, SLOT(openImages()));

	// setup the OpenGL widget
	glWidget = new GLWidget3D(this);
	//setCentralWidget(glWidget);

	QWidget* w = new QWidget(this);
	this->centralWidget()->setLayout(new QHBoxLayout());
	this->centralWidget()->layout()->addWidget(w);
	this->centralWidget()->layout()->addWidget(glWidget);

	w->setFixedSize(400, 500);
	w->setLayout(new QVBoxLayout);
	for (int i = 0; i < NUM_IMAGES; ++i) {
		w->layout()->addWidget(&imgWidget[i]);
		imgWidget[i].setFixedSize(400, 250);
	}
}

MainWindow::~MainWindow() {
}

void MainWindow::openImages() {
	for (int i = 0; i < NUM_IMAGES; ++i) {
		QString filename = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Image files (*.jpg)"));
		if (filename.isEmpty()) return;

		imgWidget[i].setImage(filename);
	}
	
	// カメラ内部パラメータ、歪み係数、外部パラメータの推定
	std::vector<std::vector<cv::Point3f> > objectPoints;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	for (int i = 0; i < NUM_IMAGES; ++i) {
		objectPoints.push_back(imgWidget[i].objectPoints);
		imagePoints.push_back(imgWidget[i].imagePoints);
	}
	glWidget->cameraMat = cv::Mat::eye(3, 3, CV_64F);
	glWidget->distortion = cv::Mat::zeros(1, 8, CV_64F);
	std::vector<cv::Mat> rvecs, tvecs;
	double totalError = Calibration::calibrateCamera(objectPoints, imagePoints, imgWidget[0].imgMat.size(), glWidget->cameraMat, glWidget->distortion, glWidget->rvecs, glWidget->tvecs);
	printf("Total error: %lf\n", totalError);
	ui.statusBar->showMessage(QString("Total error: %1").arg(totalError));

	for (int i = 0; i < NUM_IMAGES; ++i) {
		Calibration::projectPoints(imgWidget[i].objectPoints, glWidget->rvecs[i], glWidget->tvecs[i], glWidget->cameraMat, glWidget->distortion, imgWidget[i].projectedImagePoints);
	}

	glWidget->initialized = true;
	glWidget->updateGL();
}
