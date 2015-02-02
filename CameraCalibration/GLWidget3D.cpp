#include <iostream>
#include "GLWidget3D.h"
#include "MainWindow.h"
#include <GL/GLU.h>

#define SQR(x)	((x) * (x))

GLWidget3D::GLWidget3D() {
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
			float dist2 = SQR(pt.x() - c * 21.7) + SQR(pt.y() - r * 21.7);
			if (dist2 < min_dist2) {
				min_dist2 = dist2;
				selected = true;
				min_r = r;
				min_c = c;
			}
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
	glVertex3f(0, 0, -500);
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
				glPointSize(10);
				glColor3f(0, 0, 1);
			} else {
				glColor3f(1, 1, 0);
			}
			glBegin(GL_POINTS);
			glVertex3f(c * 21.7, r * 21.7, 1);
			glEnd();
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

		// カメラ１の座標を計算
		cv::Mat dst;
		cv::Rodrigues(rvecs[0], dst);
		std::vector<cv::Mat> unprojected_c1(4);
		for (int i = 0; i < 4; ++i) {
			unprojected_c1[i] = dst.inv() * (c[i] - tvecs[0]);
		}

		// カメラ１の表示
		glLineWidth(3);
		for (int i = 1; i <= 3; ++i) {
			if (i == 1) {
				glColor3f(0, 0, 1);
			} else if (i == 20) {
				glColor3f(0, 1, 0);
			} else {
				glColor3f(1, 0, 0);
			}
			glBegin(GL_LINES);
			glVertex3f(unprojected_c1[0].at<double>(0, 0), unprojected_c1[0].at<double>(1, 0), -unprojected_c1[0].at<double>(2, 0));
			glVertex3f(unprojected_c1[i].at<double>(0, 0), unprojected_c1[i].at<double>(1, 0), -unprojected_c1[i].at<double>(2, 0));
			glEnd();
		}

		std::vector<cv::Point3f> pts;
		std::vector<cv::Point2f> imgPts;
		for (int i = 0; i < 4; ++i) {
			pts.push_back(cv::Point3f(unprojected_c1[i].at<double>(0, 0), unprojected_c1[i].at<double>(1, 0), unprojected_c1[i].at<double>(2, 0)));
		}
		cv::projectPoints(pts, rvecs[0], tvecs[0], cameraMat, distortion, imgPts);
		


		// カメラ２の座標を計算
		//cv::Mat dst;
		cv::Rodrigues(rvecs[1], dst);
		std::vector<cv::Mat> unprojected_c2(4);
		for (int i = 0; i < 4; ++i) {
			unprojected_c2[i] = dst.inv() * (c[i] - tvecs[1]);
		}


		// カメラ２の表示
		glLineWidth(3);
		for (int i = 1; i <= 3; ++i) {
			if (i == 1) {
				glColor3f(0, 0, 1);
			} else if (i == 20) {
				glColor3f(0, 1, 0);
			} else {
				glColor3f(1, 0, 0);
			}
			glBegin(GL_LINES);
			glVertex3f(unprojected_c2[0].at<double>(0, 0), unprojected_c2[0].at<double>(1, 0), -unprojected_c2[0].at<double>(2, 0));
			glVertex3f(unprojected_c2[i].at<double>(0, 0), unprojected_c2[i].at<double>(1, 0), -unprojected_c2[i].at<double>(2, 0));
			glEnd();
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

	// retrieve the projected z-buffer of the origin
	GLdouble origX, origY, origZ;
	gluProject(0, 0, 0, modelview, projection, viewport, &origX, &origY, &origZ);

	// set up the projected point
	GLfloat winX = (float)x;
	GLfloat winY = (float)viewport[3] - (float)y;
	GLfloat winZ = origZ;
	
	// unproject the image plane coordinate to the model space
	GLdouble posX, posY, posZ;
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

	return QVector2D(posX, posY);
}
