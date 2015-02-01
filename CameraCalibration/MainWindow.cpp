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

	w->setFixedSize(400, 400);
	w->setLayout(new QVBoxLayout);
	w->layout()->addWidget(&imgWidget[0]);
	w->layout()->addWidget(&imgWidget[1]);

	imgWidget[0].setFixedSize(400, 400);
	imgWidget[1].setFixedSize(400, 400);
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
}
