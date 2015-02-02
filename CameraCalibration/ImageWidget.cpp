#include "ImageWidget.h"
#include <QPainter>

#define SQR(x)	((x) * (x))

ImageWidget::ImageWidget(QWidget* parent) : QWidget(parent) {
	selected = false;
}

void ImageWidget::setImage(const QString& filename) {
	imgMat = cv::imread(filename.toUtf8().data(), CV_LOAD_IMAGE_COLOR);
	cv::cvtColor(imgMat, imgMat, CV_BGR2RGB);

	// ３Ｄ座標のセット
	for (int r = 0; r < 7; ++r) {
		for (int c = 0; c < 10; ++c) {
			objectPoints.push_back(cv::Point3f(c * 21.7, (6-r) * 21.7, 0.0f));
		}
	}

	// コーナー検出
	if (cv::findChessboardCorners(imgMat, cv::Size(10, 7), imagePoints)) {
		fprintf (stderr, "ok\n");
	} else {
		fprintf (stderr, "fail\n");
	}

	// サブピクセル精度のコーナー検出
	cv::Mat grayMat(imgMat.size(), CV_8UC1);
	cv::cvtColor(imgMat, grayMat, CV_RGB2GRAY);
	cv::cornerSubPix(grayMat, imagePoints, cv::Size(3, 3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

	for (int i = 0; i < imagePoints.size(); ++i) {
		imagePoints[i].y = imgMat.rows - imagePoints[i].y;
	}

	// QImageの生成
	image = QImage(imgMat.data, imgMat.cols, imgMat.rows, QImage::Format_RGB888);

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

void ImageWidget::selectPoint(int index) {
	selected = true;
	selected_index = index;

	update();
}

void ImageWidget::paintEvent(QPaintEvent * /* event */) {
	QPainter painter(this);

	painter.scale(scale, scale);

	painter.drawImage(QPoint(0, 0), image);

	// 検出したコーナーを表示
	for (int i = 0; i < imagePoints.size(); ++i) {
		if (selected && i == selected_index) {
			painter.setPen(QPen(QColor(0, 0, 255), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		} else {
			painter.setPen(QPen(QColor(255, 255, 0), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		}
		painter.drawEllipse(imagePoints[i].x-6, image.height()-imagePoints[i].y-6, 12, 12);
	}

	// 誤差を表示
	if (selected) {
		double error = sqrtf(SQR(projectedImagePoints[selected_index].x - imagePoints[selected_index].x) + SQR(projectedImagePoints[selected_index].y - imagePoints[selected_index].y));

		painter.setPen(QPen(QColor(255, 255, 0), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		painter.setFont(QFont("Selif", 20));
		QString str = QString("Error: %1").arg(error);
		painter.drawText(8, 25, str);
	}
}

void ImageWidget::mousePressEvent(QMouseEvent *event) {
	/*if (event->button() == Qt::LeftButton) {
		drawPoint(event->pos());
	}*/
}