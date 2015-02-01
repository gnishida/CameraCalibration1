#include <iostream>
#include "GLWidget3D.h"
#include "MainWindow.h"
#include <GL/GLU.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

GLWidget3D::GLWidget3D() {
	// set up the camera
	camera.setLookAt(0.0f, 0.0f, 0.0f);
	camera.setYRotation(0);
	camera.setTranslation(0.0f, 0.0f, 1000.0f);
}

/**
 * This event handler is called when the mouse press events occur.
 */
void GLWidget3D::mousePressEvent(QMouseEvent *e)
{
	lastPos = e->pos();
}

/**
 * This event handler is called when the mouse release events occur.
 */
void GLWidget3D::mouseReleaseEvent(QMouseEvent *e)
{
	updateGL();
}

/**
 * This event handler is called when the mouse move events occur.
 */
void GLWidget3D::mouseMoveEvent(QMouseEvent *e)
{
	float dx = (float)(e->x() - lastPos.x());
	float dy = (float)(e->y() - lastPos.y());
	lastPos = e->pos();

	if (e->buttons() & Qt::LeftButton) {
		camera.changeXRotation(dy);
		camera.changeYRotation(dx);
	} else if (e->buttons() & Qt::RightButton) {
		camera.changeXYZTranslation(0, 0, -dy * camera.dz * 0.02f);
		if (camera.dz < -9000) camera.dz = -9000;
		if (camera.dz > 9000) camera.dz = 9000;
	}

	updateGL();
}

/**
 * This function is called once before the first call to paintGL() or resizeGL().
 */
void GLWidget3D::initializeGL()
{
	glClearColor(0.443, 0.439, 0.458, 0.0);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);

	static GLfloat lightPosition[4] = {0.0f, 0.0f, 100.0f, 0.0f};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

	QImage img;
	img.load("checkerboard.jpg");
	texture = bindTexture(QImage("checkerboard.jpg"));
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

/**
 * This function is called whenever the widget has been resized.
 */
void GLWidget3D::resizeGL(int width, int height)
{
	height = height?height:1;

	glViewport( 0, 0, (GLint)width, (GLint)height );
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)width/(GLfloat)height, 0.1f, 10000);
	glMatrixMode(GL_MODELVIEW);
}

/**
 * This function is called whenever the widget needs to be painted.
 */
void GLWidget3D::paintGL()
{
	glMatrixMode(GL_MODELVIEW);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	camera.applyCamTransform();

	drawScene();		
}

/**
 * Draw the scene.
 */
void GLWidget3D::drawScene() {
	// カメラの内部パラメータ行列
	cv::Mat A = cv::Mat::eye(4, 4, CV_64F);
	A.at<double>(0, 0) = 2644.539;
	A.at<double>(0, 2) = 1117.647;
	A.at<double>(1, 1) = 2606.132;
	A.at<double>(1, 2) = 638.9;
	A.at<double>(2, 2) = 1;

	// カメラ１の外部パラメータ行列
	cv::Mat P1 = cv::Mat::eye(4, 4, CV_64F);
	P1.at<double>(0, 0) = 0.836;
	P1.at<double>(0, 1) = 0.011;
	P1.at<double>(0, 2) = 0.549;
	P1.at<double>(0, 3) = -147.315;
	P1.at<double>(1, 0) = 0.021;
	P1.at<double>(1, 1) = 0.998;
	P1.at<double>(1, 2) = -0.052;
	P1.at<double>(1, 3) = -75.438;
	P1.at<double>(2, 0) = -0.548;
	P1.at<double>(2, 1) = 0.054;
	P1.at<double>(2, 2) = 0.835;
	P1.at<double>(2, 3) = 952.71;

	// カメラ２の外部パラメータ行列
	cv::Mat P2 = cv::Mat::eye(4, 4, CV_64F);
	P2.at<double>(0, 0) = 0.926;
	P2.at<double>(0, 1) = 0.003;
	P2.at<double>(0, 2) = -0.378;
	P2.at<double>(0, 3) = -56.936;
	P2.at<double>(1, 0) = -0.011;
	P2.at<double>(1, 1) = 1.000;
	P2.at<double>(1, 2) = -0.020;
	P2.at<double>(1, 3) = -56.468;
	P2.at<double>(2, 0) = 0.377;
	P2.at<double>(2, 1) = 0.023;
	P2.at<double>(2, 2) = 0.926;
	P2.at<double>(2, 3) = 940.070;

	// ワールド座標系の軸表示
	glPointSize(3);
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(500, 0, 0);
	glEnd();
	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 500, 0);
	glEnd();
	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 500);
	glEnd();

	// チェックボードの表示
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture);
	glBegin(GL_QUADS);
	glColor3f(1, 1, 1);
	glNormal3f(0, 0, 1);
	glTexCoord2f(0, 0);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1, 0);
	glVertex3f(216, 0, 0);
	glTexCoord2f(1, 1);
	glVertex3f(216, 144, 0);
	glTexCoord2f(0, 1);
	glVertex3f(0, 144, 0);
	glEnd();
	glDisable(GL_TEXTURE_2D);


	// カメラの座標
	cv::Mat c(4, 4, CV_64F, cv::Scalar(0));
	c.at<double>(0, 0) = 0.0f;
	c.at<double>(1, 0) = 0.0f;
	c.at<double>(2, 0) = 0.0f;
	c.at<double>(3, 0) = 1.0f;
	c.at<double>(0, 1) = 2304;
	c.at<double>(1, 1) = 0;
	c.at<double>(2, 1) = 0;
	c.at<double>(3, 1) = 1.0f;
	c.at<double>(0, 2) = 2304;
	c.at<double>(1, 2) = 1296;
	c.at<double>(2, 2) = 0;
	c.at<double>(3, 2) = 1.0f;
	c.at<double>(0, 3) = 0;
	c.at<double>(1, 3) = 1296;
	c.at<double>(2, 3) = 0;
	c.at<double>(3, 3) = 1.0f;

	// カメラ１の座標を計算
	cv::Mat unprojected_c1 = P1.inv() * A.inv() * c;

	/*printf("%.3lf, %.3lf, %.3lf\n", unprojected_c1.at<double>(0, 0), unprojected_c1.at<double>(1, 0), unprojected_c1.at<double>(2, 0));
	printf("%.3lf, %.3lf, %.3lf\n", unprojected_c1.at<double>(0, 1), unprojected_c1.at<double>(1, 1), unprojected_c1.at<double>(2, 1));
	printf("%.3lf, %.3lf, %.3lf\n", unprojected_c1.at<double>(0, 2), unprojected_c1.at<double>(1, 2), unprojected_c1.at<double>(2, 2));
	*/

	// カメラ１の表示
	glLineWidth(3);
	glColor3f(0, 0, 1);
	for (int i = 0; i < 4; ++i) {
		int next = (i + 1) % 4;
		glBegin(GL_LINES);
		glVertex3f(unprojected_c1.at<double>(0, i), unprojected_c1.at<double>(1, i), -unprojected_c1.at<double>(2, i));
		glVertex3f(unprojected_c1.at<double>(0, next), unprojected_c1.at<double>(1, next), -unprojected_c1.at<double>(2, next));
		glEnd();
	}

	glBegin(GL_POINTS);
	glVertex3f(unprojected_c1.at<double>(0, 0), unprojected_c1.at<double>(1, 0), -unprojected_c1.at<double>(2, 0));
	glEnd();


	// カメラ２の座標を計算
	cv::Mat unprojected_c2 = P2.inv() * A.inv() * c;

	// カメラ１の表示
	glLineWidth(3);
	glColor3f(0, 0, 1);
	for (int i = 0; i < 4; ++i) {
		int next = (i + 1) % 4;
		glBegin(GL_LINES);
		glVertex3f(unprojected_c2.at<double>(0, i), unprojected_c2.at<double>(1, i), -unprojected_c2.at<double>(2, i));
		glVertex3f(unprojected_c2.at<double>(0, next), unprojected_c2.at<double>(1, next), -unprojected_c2.at<double>(2, next));
		glEnd();
	}

	glBegin(GL_POINTS);
	glVertex3f(unprojected_c2.at<double>(0, 0), unprojected_c2.at<double>(1, 0), -unprojected_c2.at<double>(2, 0));
	glEnd();
}

