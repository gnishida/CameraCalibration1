#include <iostream>
#include "GLWidget3D.h"
#include "MainWindow.h"
#include <GL/GLU.h>

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
void GLWidget3D::drawScene()
{
	glRotatef(-1.527, 0, 0, 1);
	glRotatef(-0.411, 0, 1, 0);
	glRotatef(-0.499, 1, 0, 0);
	glTranslatef(147, 75, -952);

	glColor3f(1, 0, 0);
	glBegin(GL_QUADS);
	glVertex3f(0, 0, 0);
	glVertex3f(0, -144, 0);
	glVertex3f(216, -144, 0);
	glVertex3f(216, 0, 0);
	glEnd();
}


