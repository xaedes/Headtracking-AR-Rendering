#pragma once

#include <QMouseEvent>
#include <QWheelEvent>
#include <opencv2/opencv.hpp>

namespace ar {
namespace Gui {

	class Keyboard;

	class Camera
	{
	public:
		Camera(Keyboard& keyboard);
		~Camera();
		void setProjection(int display_width, int display_height);
		void setView();
		void setRotationCenter(float x, float y, float z);
		void getRotationCenter(float& x, float& y, float& z);

		void setYaw(float yaw);
		void setPitch(float pitch);
		void setObserverPose(cv::Vec6d pose);

		void mousePressEvent(QMouseEvent *event);
		void mouseReleaseEvent(QMouseEvent *event);
		void mouseMoveEvent(QMouseEvent *event);
		void wheelEvent(QWheelEvent *event);
	protected:
		Keyboard& m_keyboard;
		float m_x;
		float m_y;
		float m_zoom;
		float m_yaw;
		float m_pitch;

		float m_rotCenterX;
		float m_rotCenterY;
		float m_rotCenterZ;

		int m_last_x;
		int m_last_y;

		bool m_dragging;
	};

} // namespace Gui
} // namespace ar
