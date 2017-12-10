#include "Camera.h"
#include "common/math.h"
#include "Keyboard.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>

namespace ar {
namespace Gui {

	Camera::Camera(Keyboard& keyboard) :
		m_keyboard(keyboard),
		m_x(0),
		m_y(0),
		m_zoom(100),
		m_yaw(23),
		m_pitch(-44),
		m_last_x(0),
		m_last_y(0),
		m_dragging(false),
		m_rotCenterX(0),
		m_rotCenterY(0),
		m_rotCenterZ(0)
	{}

	Camera::~Camera()
	{}

	void Camera::setProjection(int display_width, int display_height)
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		int x_min, x_max, y_min, y_max;
		
		x_min = 0 -(display_width/2);
		y_min = 0 -(display_height/2);
		x_max = x_min + display_width;
		y_max = y_min + display_height;

		glOrtho(x_min,x_max,y_min,y_max,-10000,10000);
	}

	void Camera::setGeneralizedProjection(
		const cv::Vec3f& screenCornerBottomLeft,
		const cv::Vec3f& screenCornerBottomRight,
		const cv::Vec3f& screenCornerTopLeft,
		const cv::Vec3f& observerPosition,
		float nearPlane,
		float farPlane
	)
	{
		// Generalized Perspective Projection. Robert Kooima, 2008.
		// http://csc.lsu.edu/~kooima/pdfs/gen-perspective.pdf
		
		// compute orthonormal basis for the screen
		cv::Vec3f vRight = screenCornerBottomRight - screenCornerBottomLeft;
		cv::Vec3f vUp = screenCornerTopLeft - screenCornerBottomLeft;
		vRight = vRight / cv::norm(vRight);
		vUp = vUp / cv::norm(vUp);
		cv::Vec3f vNormal = vRight.cross(vUp);
		vNormal = vNormal / cv::norm(vNormal);
		
		// compute relative screen corner vectors
		cv::Vec3f a = screenCornerBottomLeft - observerPosition;
		cv::Vec3f b = screenCornerBottomRight - observerPosition;
		cv::Vec3f c = screenCornerTopLeft - observerPosition;

		// find the distance from the observer to screen plane
		float d = -a.dot(vNormal);

		// find the extent of the perpendicular projection
		float left   = vRight.dot(a) * nearPlane / d;
		float right  = vRight.dot(b) * nearPlane / d;
		float bottom = vUp.dot(a) * nearPlane / d;
		float top    = vUp.dot(c) * nearPlane / d;

		// load the perpendicular projection
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glFrustum(left, right, bottom, top, nearPlane, farPlane);

		// rotate the projection to be non-perpendicular
		cv::Matx44f rotation(
			vRight[0] , vRight[1] , vRight[2] , 0,
			vUp[0]    , vUp[1]    , vUp[2]    , 0,
			vNormal[0], vNormal[1], vNormal[2], 0,
			0         , 0         , 0         , 1
		);
		glMultMatrixf(&rotation(0,0));

		// move the apex of the frustum to the origin
		glTranslatef(-observerPosition[0],-observerPosition[1],-observerPosition[2]);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
	}

	void Camera::setView()
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glScalef(m_zoom, m_zoom, 1);
		glTranslatef(-m_x,-m_y,0);

		glRotatef(m_pitch,1,0,0);
		glRotatef(m_yaw,0,0,1);
		glTranslatef(-m_rotCenterX,-m_rotCenterY,-m_rotCenterZ);
	}

	void Camera::setRotationCenter(float x, float y, float z)
	{
		m_rotCenterX = x;
		m_rotCenterY = y;
		m_rotCenterZ = z;
	}

	void Camera::getRotationCenter(float& x, float& y, float& z)
	{
		x = m_rotCenterX;
		y = m_rotCenterY;
		z = m_rotCenterZ;
	}

	void Camera::setYaw(float yaw)
	{
		m_yaw = yaw;
	}
	void Camera::setPitch(float pitch)
	{
		m_pitch = pitch;
	}
	void Camera::setObserverPose(cv::Vec6d pose)
	{
        double yaw = atan2(pose[2],pose[0]);
        double pitch = atan2(pose[2],pose[1]);
        setYaw(-yaw * RADIAN_TO_DEGREE);
        setPitch(pitch * RADIAN_TO_DEGREE);
    }

	void Camera::mousePressEvent(QMouseEvent *event)
	{
		m_last_x = event->x();
		m_last_y = event->y();
		m_dragging = true;
	}
	
	void Camera::mouseReleaseEvent(QMouseEvent *event)
	{
		if (m_dragging)
		{
			m_dragging = false;
		}
	}
	
	void Camera::mouseMoveEvent(QMouseEvent *event)
	{
		if (m_dragging)
		{
			Qt::MouseButtons buttons = event->buttons();
			int dx = event->x() - m_last_x;
			int dy = event->y() - m_last_y;

			if (buttons & Qt::LeftButton)
			{
				m_x -= dx / m_zoom;
				m_y -= (-dy) / m_zoom;  // -dy because image y-axis is inverse to world y-axis
			}
			else if (buttons & Qt::RightButton)
			{
				m_yaw   += dx;
				m_pitch += dy;
			}


			m_last_x = event->x();
			m_last_y = event->y();			
		}
	}

	void Camera::wheelEvent(QWheelEvent *event)
	{

		// QWheelEvent.angleDelta().y() Returns the distance that the wheel is rotated, in eighths of a degree.

		float deltaDeg = ((float)event->angleDelta().y())/8.;
		//float z_per_deg = 1 / 15.
		if (deltaDeg > 0)
		{
			m_zoom *= 1.1;
		}
		else if (deltaDeg < 0)
		{
			m_zoom *= 0.9;
		}

	}
	

} // namespace Gui
} // namespace ar
