#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include "ui_MainWindow.h"
#include "GLWidget3D.h"
#include "ImageWidget.h"

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0, Qt::WFlags flags = 0);
	~MainWindow();

public slots:
	void openImages();

private:
	Ui::MainWindowClass ui;
	GLWidget3D* glWidget;
	ImageWidget imgWidget[2];
};

#endif // MAINWINDOW_H
