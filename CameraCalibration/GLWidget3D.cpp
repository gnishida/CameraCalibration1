﻿#include <iostream>
#include "GLWidget3D.h"
#include "MainWindow.h"
#include <GL/GLU.h>
#include "Calibration.h"

#define SQR(x)	((x) * (x))

GLWidget3D::GLWidget3D(MainWindow* mainWin) {
	this->mainWin = mainWin;

	// set up the camera
	camera.setLookAt(0.0f, 0.0f, 0.0f);
	camera.setYRotation(0);
	camera.setTranslation(0.0f, 0.0f, 1500.0f);

	initialized = false;
}

/**
 * This event handler is called when the mouse press events occur.
 */
void GLWidget3D::mousePressEvent(QMouseEvent *e)
{
	lastPos = e->pos();

	// チェックボード上のコーナーを選択
	QVector2D pt = mouseTo2D(e->x(), e->y());
	float min_dist2 = 100;
	selected = false;
	for (int r = 0; r < 7; ++r) {
		for (int c = 0; c < 10; ++c) {
			float dist2 = SQR(pt.x() - c * 21.7) + SQR(pt.y() - (6-r) * 21.7);
			if (dist2 < min_dist2) {
				min_dist2 = dist2;
				selected = true;
				min_r = r;
				min_c = c;
			}
		}
	}

	// 各2D画像上でも、該当の点を表示し、エラーも表示する
	if (initialized && selected) {
		for (int i = 0; i < NUM_IMAGES; ++i) {
			mainWin->imgWidget[i].selectPoint(min_r * 10 + min_c);
		}
	}
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
	} else if (e->buttons() & Qt::MidButton) {
		camera.changeXYZTranslation(-dx, dy, 0);
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
	glVertex3f(-21.7, -21.7, 0);

	glTexCoord2f(1, 0);
	glVertex3f(217, -21.7, 0);

	glTexCoord2f(1, 1);
	glVertex3f(217, 21.7*7, 0);

	glTexCoord2f(0, 1);
	glVertex3f(-21.7, 21.7*7, 0);

	glEnd();
	glDisable(GL_TEXTURE_2D);

	// チェックボード上のコーナーを表示
	for (int r = 0; r < 7; ++r) {
		for (int c = 0; c < 10; ++c) {
			glPointSize(6);
			if (selected && r == min_r && c == min_c) {
				drawSphere(c * 21.7, (6-r) * 21.7, 0, 4, QColor(0, 0, 255));
			} else {
				drawSphere(c * 21.7, (6-r) * 21.7, 0, 4, QColor(255, 255, 0));
			}
		}
	}

	if (initialized) {
		// カメラの座標
		std::vector<cv::Mat> c(4);
		c[0] = cv::Mat(3, 1, CV_64F, cv::Scalar(0));
		c[0].at<double>(0, 0) = 0;
		c[0].at<double>(1, 0) = 0;
		c[0].at<double>(2, 0) = 0;
		c[1] = cv::Mat(3, 1, CV_64F, cv::Scalar(0));
		c[1].at<double>(0, 0) = 100;
		c[1].at<double>(1, 0) = 0;
		c[1].at<double>(2, 0) = 0;
		c[2] = cv::Mat(3, 1, CV_64F, cv::Scalar(0));
		c[2].at<double>(0, 0) = 0;
		c[2].at<double>(1, 0) = 100;
		c[2].at<double>(2, 0) = 0;
		c[3] = cv::Mat(3, 1, CV_64F, cv::Scalar(0));
		c[3].at<double>(0, 0) = 0;
		c[3].at<double>(1, 0) = 0;
		c[3].at<double>(2, 0) = 100;

		for (int i = 0; i < NUM_IMAGES; ++i) {
			// カメラ座標を計算
			cv::Mat dst;
			cv::Rodrigues(rvecs[i], dst);
			std::vector<cv::Mat> unprojected_c(4);
			for (int k = 0; k < 4; ++k) {
				unprojected_c[k] = dst.inv() * (c[k] - tvecs[i]);
			}

			// カメラの表示
			glLineWidth(3);
			for (int k = 1; k <= 3; ++k) {
				if (k == 1) {
					glColor3f(0, 0, 1);
				} else if (k == 2) {
					glColor3f(0, 1, 0);
				} else {
					glColor3f(1, 0, 0);
				}
				glBegin(GL_LINES);
				glVertex3f(unprojected_c[0].at<double>(0, 0), unprojected_c[0].at<double>(1, 0), -unprojected_c[0].at<double>(2, 0));
				glVertex3f(unprojected_c[k].at<double>(0, 0), unprojected_c[k].at<double>(1, 0), -unprojected_c[k].at<double>(2, 0));
				glEnd();
			}
		}
	}
}

QVector2D GLWidget3D::mouseTo2D(int x,int y) {
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];

	// retrieve the matrices
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	float z;
	glReadPixels(x, (float)viewport[3] - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
	
	// unproject the image plane coordinate to the model space
	GLdouble posX, posY, posZ;
	gluUnProject(x, (float)viewport[3] - y, z, modelview, projection, viewport, &posX, &posY, &posZ);

	return QVector2D(posX, posY);
}

void GLWidget3D::drawSphere(float x, float y, float z, float r, const QColor& color) {
	int slices = 16;
	int stacks = 8;

	glBegin(GL_QUADS);
	glColor3f(color.redF(), color.greenF(), color.blueF());
	for (int i = 0; i < slices; ++i) {
		float theta1 = M_PI * 2.0f / slices * i;
		float theta2 = M_PI * 2.0f / slices * (i + 1);

		for (int j = 0; j < stacks; ++j) {
			float phi1 = M_PI / stacks * j - M_PI * 0.5;
			float phi2 = M_PI / stacks * (j + 1) - M_PI * 0.5;

			QVector3D pt1 = QVector3D(cosf(theta1) * cosf(phi1), sinf(theta1) * cosf(phi1), sinf(phi1));
			QVector3D pt2 = QVector3D(cosf(theta2) * cosf(phi1), sinf(theta2) * cosf(phi1), sinf(phi1));
			QVector3D pt3 = QVector3D(cosf(theta2) * cosf(phi2), sinf(theta2) * cosf(phi2), sinf(phi2));
			QVector3D pt4 = QVector3D(cosf(theta1) * cosf(phi2), sinf(theta1) * cosf(phi2), sinf(phi2));

			glNormal3f(pt1.x(), pt1.y(), pt1.z());
			glVertex3f(x + pt1.x() * r, y + pt1.y() * r, z + pt1.z() * r);
			glNormal3f(pt2.x(), pt2.y(), pt2.z());
			glVertex3f(x + pt2.x() * r, y + pt2.y() * r, z + pt2.z() * r);
			glNormal3f(pt3.x(), pt3.y(), pt3.z());
			glVertex3f(x + pt3.x() * r, y + pt3.y() * r, z + pt3.z() * r);
			glNormal3f(pt4.x(), pt4.y(), pt4.z());
			glVertex3f(x + pt4.x() * r, y + pt4.y() * r, z + pt4.z() * r);
		}
	}
	glEnd();
}
