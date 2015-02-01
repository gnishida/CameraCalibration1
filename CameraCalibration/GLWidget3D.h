#pragma once

#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include "Camera.h"
#include <QVector3D>
#include <vector>

using namespace std;

class GLWidget3D : public QGLWidget
{
public:
	GLWidget3D();
	void drawScene();
	QVector2D mouseTo2D(int x,int y);

protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();    
	void mousePressEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);

private:
	static enum{VERTEX,NORMAL,COLOR,TOTAL_VBO_ID};

	Camera camera;
	QPoint lastPos;
	GLuint texture;

	bool selected;
	int min_r;
	int min_c;
};

