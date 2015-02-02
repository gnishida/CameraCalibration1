#pragma once

#include <QWidget>
#include <QPixmap>
#include <QMouseEvent>
#include <opencv/cv.h>
#include <opencv/highgui.h>

class ImageWidget : public QWidget {
Q_OBJECT

private:
	QImage image;
	float scale;
	bool selected;
	int selected_index;

public:
	cv::Mat imgMat;
	std::vector<cv::Point3f> objectPoints;
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point2f> projectedImagePoints;

public:
	ImageWidget(QWidget* parent = 0);

	void setImage(const QString& filename);
	void drawPoint(const QPoint &point);
	void selectPoint(int index);

protected:
	void paintEvent(QPaintEvent* event);
	void mousePressEvent(QMouseEvent* event);
};

