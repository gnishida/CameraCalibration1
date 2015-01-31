#include <iostream>
#include "GLWidget3D.h"
#include "MainWindow.h"
#include <GL/GLU.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

GLWidget3D::GLWidget3D() {
	patVertices.push_back(QVector3D(0, 0, 0));
	patVertices.push_back(QVector3D(216, 0, 0));
	patVertices.push_back(QVector3D(216, 144, 0));

	patVertices.push_back(QVector3D(0, 0, 0));
	patVertices.push_back(QVector3D(216, 144, 0));
	patVertices.push_back(QVector3D(0, 144, 0));
}

/**
 * This event handler is called when the mouse press events occur.
 */
void GLWidget3D::mousePressEvent(QMouseEvent *e)
{
	camera.mouseDown(e->x(), e->y());
}

/**
 * This event handler is called when the mouse release events occur.
 */
void GLWidget3D::mouseReleaseEvent(QMouseEvent *e)
{
	camera.mouseUp();

	updateGL();
}

/**
 * This event handler is called when the mouse move events occur.
 */
void GLWidget3D::mouseMoveEvent(QMouseEvent *e)
{
	if (e->buttons() & Qt::LeftButton) {
		camera.rotate(e->x(), e->y());
	} else if (e->buttons() & Qt::RightButton) {
		camera.zoom(e->x(), e->y());
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
}

/**
 * This function is called whenever the widget has been resized.
 */
void GLWidget3D::resizeGL(int width, int height)
{
	height = height?height:1;

	camera.setWindowSize(width, height);

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

    glLoadIdentity();
    glTranslatef(0.0, 0.0, -camera.z);
	glMultMatrixd(camera.rt);

	drawScene();		
}

/**
 * Draw the scene.
 */
void GLWidget3D::drawScene() {
	cv::Mat A = cv::Mat::eye(4, 4, CV_64F);
	cv::Mat P = cv::Mat::eye(4, 4, CV_64F);
	A.at<double>(0, 0) = 2644.539;
	A.at<double>(0, 2) = 1117.647;
	A.at<double>(1, 1) = 2606.132;
	A.at<double>(1, 2) = 638.9;
	A.at<double>(2, 2) = 1;

	P.at<double>(0, 0) = 0.836;
	P.at<double>(0, 1) = 0.011;
	P.at<double>(0, 2) = 0.549;
	P.at<double>(0, 3) = -147.315;
	P.at<double>(1, 0) = 0.021;
	P.at<double>(1, 1) = 0.998;
	P.at<double>(1, 2) = -0.052;
	P.at<double>(1, 3) = -75.438;
	P.at<double>(2, 0) = -0.548;
	P.at<double>(2, 1) = 0.054;
	P.at<double>(2, 2) = 0.835;
	P.at<double>(2, 3) = 952.71;

	cv::Mat c1(4, 1, CV_64F, cv::Scalar(0));
	c1.at<double>(3, 0) = 1.0f;
	cv::Mat invA = A.inv();
	cv::Mat invP = P.inv();
	cv::Mat unprojected_c1 = P.inv() * A.inv() * c1;

	printf("%.3lf, %.3lf, %.3lf\n", unprojected_c1.at<double>(0, 0), unprojected_c1.at<double>(1, 0), unprojected_c1.at<double>(2, 0));

	glColor3f(1, 0, 0);
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < patVertices.size() / 3; ++i) {
		glVertex3f(patVertices[i * 3 + 0].x(), patVertices[i * 3 + 0].y(), patVertices[i * 3 + 0].z());
		glVertex3f(patVertices[i * 3 + 1].x(), patVertices[i * 3 + 1].y(), patVertices[i * 3 + 1].z());
		glVertex3f(patVertices[i * 3 + 2].x(), patVertices[i * 3 + 2].y(), patVertices[i * 3 + 2].z());
	}
	glEnd();

	glPointSize(10);
	glBegin(GL_POINTS);
	glVertex3f(unprojected_c1.at<double>(0, 0), unprojected_c1.at<double>(1, 0), unprojected_c1.at<double>(2, 0));
	glEnd();
}


