#pragma once

#include <memory>
#include <string>

#include <GL/gl.h>
#include <GL/glu.h>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QTimerEvent>
#include <QModelIndex>

#include <opencv2/opencv.hpp>

namespace ar {
namespace Gui {

	// forward declaration
	class App;
	
	class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
	{
		Q_OBJECT

	public:
		GLWidget(App& app, QWidget *parent = 0);
		~GLWidget();

		QSize minimumSizeHint() const Q_DECL_OVERRIDE;
		QSize sizeHint() const Q_DECL_OVERRIDE;

	public slots:
		void cleanup();

	protected:
		App& m_app;

	    cv::Mat_<cv::Vec3b> m_colorFrame;
	    cv::Mat_<cv::Vec3b> m_colorFrameScaled;

		void initializeGL() Q_DECL_OVERRIDE;
		void paintGL() Q_DECL_OVERRIDE;	
		void resizeGL(int width, int height) Q_DECL_OVERRIDE;

		void checkGlError();

		void paintAxes(float size);
		void paintAxes(const cv::Matx44f& coordinateSystem, float size);

		void paintGridSpotlight(const cv::Vec2f& xyPosition, float size, float interval);

		cv::Vec3f numberedColor(int k);

		virtual void timerEvent(QTimerEvent *event);
		virtual void mousePressEvent(QMouseEvent *event);
		virtual void mouseReleaseEvent(QMouseEvent *event);
		virtual void mouseMoveEvent(QMouseEvent *event);
		virtual void wheelEvent(QWheelEvent *event);
		virtual void keyPressEvent(QKeyEvent *event);
		virtual void keyReleaseEvent(QKeyEvent *event);
	};

} // namespace Gui
} // namespace ar
