#include "ImageWidget.h"
#include <QPainter>

ImageWidget::ImageWidget(QWidget* parent) : QWidget(parent) {
}

void ImageWidget::setImage(const QString& filename) {
	image.load(filename);

	scale = std::min((float)this->size().width() / image.width(), (float)this->size().height() / image.height());

	update();
}

void ImageWidget::drawPoint(const QPoint &point) {
	/*
	QPoint pt(((float)point.x() - 0.5) / scale, ((float)point.y() - 0.5) / scale);

	QPainter painter(&fgImage);
	painter.setPen(QPen(color, penWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setRenderHint(QPainter::HighQualityAntialiasing);
	painter.drawPoint(pt);
	
	int rad = (penWidth / 2) + 2;
	rad *= scale;
	update(QRect(point, point).normalized().adjusted(-rad, -rad, +rad, +rad));

	history.push_back(Stroke(Stroke::TYPE_POINT, point, point, color, penWidth));
	*/
}

void ImageWidget::paintEvent(QPaintEvent * /* event */) {
	QPainter painter(this);

	painter.scale(scale, scale);

	painter.drawImage(QPoint(0, 0), image);
}

void ImageWidget::mousePressEvent(QMouseEvent *event) {
	if (event->button() == Qt::LeftButton) {
		drawPoint(event->pos());
	}
}