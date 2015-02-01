#pragma once

#include <QWidget>
#include <QPixmap>
#include <QMouseEvent>

class ImageWidget : public QWidget {
Q_OBJECT

public:
	ImageWidget(QWidget* parent = 0);

	void setImage(const QString& filename);
	void drawPoint(const QPoint &point);

protected:
	void paintEvent(QPaintEvent* event);
	void mousePressEvent(QMouseEvent* event);

private:
	QImage image;
	float scale;
};

