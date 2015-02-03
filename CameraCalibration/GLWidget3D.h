#pragma once

#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include "Camera.h"
#include <QVector3D>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

class MainWindow;

class GLWidget3D : public QGLWidget {
private:
	MainWindow* mainWin;
	Camera camera;
	QPoint lastPos;
	GLuint texture;

	bool selected;
	int min_r;
	int min_c;

public:
	bool initialized;
	std::vector<cv::Mat> rvecs, tvecs;
	cv::Mat cameraMat;
	cv::Mat distortion;

public:
	GLWidget3D(MainWindow* mainWin);
	void drawScene();
	QVector2D mouseTo2D(int x,int y);
	void drawSphere(float x, float y, float z, float r, const QColor& color);

protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();    
	void mousePressEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);

};

