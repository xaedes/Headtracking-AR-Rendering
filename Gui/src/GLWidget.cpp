#include <QCoreApplication>
#include <QOpenGLContext>

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QStatusBar>
#include <vector>


#include <iostream>
#include <string>

#include "GLWidget.h"
#include "App.h"
#include "Keyboard.h"
#include "KeyboardControl.h"
#include "Mainwindow.h"
#include "Camera.h"

#include "common/math.h"


using namespace Qt;
using namespace std;

namespace ar {
namespace Gui {



	GLWidget::GLWidget(App& app, QWidget *parent) :
		QOpenGLWidget(parent),
		m_app(app)
	{
		setFocusPolicy(Qt::StrongFocus);
		setFocus();
		startTimer(10);
	}

	GLWidget::~GLWidget()
	{
	}



	QSize GLWidget::minimumSizeHint() const
	{
		return QSize(50, 50);
	}

	QSize GLWidget::sizeHint() const
	{
		return QSize(400, 400);
	}

	void GLWidget::cleanup()
	{
		makeCurrent();
		// do cleanup
		doneCurrent();
	}


	void GLWidget::initializeGL()
	{
		initializeOpenGLFunctions();
		// In this example the widget's corresponding top-level window can change
		// several times during the widget's lifetime. Whenever this happens, the
		// QOpenGLWidget's associated context is destroyed and a new one is created.
		// Therefore we have to be prepared to clean up the resources on the
		// aboutToBeDestroyed() signal, instead of the destructor. The emission of
		// the signal will be followed by an invocation of initializeGL() where we
		// can recreate all resources.
		connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &GLWidget::cleanup);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
		// glEnable(GL_DEPTH_TEST);
		// glDepthFunc(GL_LESS);
		glClearColor(0, 0, 0, 0);

	}

	void GLWidget::timerEvent(QTimerEvent *event)
	{
		update();
		float dt = 0.01;
		m_app.m_keyboardControl->update(dt);

        m_app.m_colorStream >> m_colorFrame;

        cv::Size size = m_colorFrame.size();
        cv::Size scaledSize((int)ceil(size.width * 2), (int)ceil(size.height * 2));
        cv::resize(m_colorFrame, m_colorFrameScaled, scaledSize);
        m_app.m_faceDetection.pushImage(m_colorFrameScaled);

        cv::Vec6d headPose = m_app.m_faceDetection.headPose();
        std::cout << "headPose " << headPose << std::endl;
        // m_app.m_camera->setObserverPose(headPose);

        cv::Mat visualizedImage = m_app.m_faceDetection.visualizedImage(m_colorFrameScaled);
        if(m_colorFrameScaled.data)
        {
            cv::imshow("m_colorFrameScaled", m_colorFrameScaled);
        }
        if(visualizedImage.data)
        {
            cv::imshow("visualized", visualizedImage);
        }
        int key = cv::waitKey(1);

		// m_vehicle.m_vehicleOrientation.m_yaw += 1 * DEGREE_TO_RADIAN;
	}

	void GLWidget::paintGL()
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Vector3D pos = m_app.m_vehicle.vehiclePosition();
		// m_app.m_camera->setRotationCenter(pos[0], pos[1], pos[2]);
		float left = 50;
		float right = -50;
		float bottom = -50;
		float top = -150;
		float zScreen = -50;

		// ganz links: (-250,120,600)
		// ganz rechts: (+270,120,600)
		// mitte oben: (0,-200,600)


		cv::Vec3f screenCornerBottomLeft(left,bottom,zScreen);
		cv::Vec3f screenCornerBottomRight(right,bottom,zScreen);
		cv::Vec3f screenCornerTopLeft(left,top,zScreen);
        cv::Vec6d headPose = m_app.m_faceDetection.headPose();
		cv::Vec3f observerPosition(
			-headPose[0],
			headPose[1],
			headPose[2]
		);

		m_app.m_camera->setGeneralizedProjection(
			screenCornerBottomLeft,
			screenCornerBottomRight,
			screenCornerTopLeft,
			observerPosition,
			1,
			2000
		);
		// m_app.m_camera->setView();

		cv::Matx44f coordinateSystem(
			1,0,0,(left+right)/2,
			0,1,0,(bottom+top)/2,
			0,0,1,zScreen,
			0,0,0,1
		);
		paintAxes(coordinateSystem,50);
		// paintAxes(3);
		
		// m_app.m_vehicle.updateCoordinateSystems();

		// TransformationMatrix3D world_masscenter = m_app.m_vehicle.world_masscenter();
		// TransformationMatrix3D world_thruster1 = m_app.m_vehicle.world_thruster1();
		// TransformationMatrix3D world_thruster2 = m_app.m_vehicle.world_thruster2();

		// paintAxes(world_masscenter, 0.1);
		// paintAxes(world_thruster1, 0.1);
		// paintAxes(world_thruster2, 0.1);

		// std::vector<RigidBody>& weights = m_app.m_vehicle.weights();
		// for(RigidBody& weight : weights)
		// {
		// 	TransformationMatrix3D world_weight = m_app.m_vehicle.world_vehicle() * weight.parent_body();
		// 	paintAxes(world_weight, 0.1);
		// }

		// glBegin(GL_LINES);
		// glColor3f(1,1,1);

		// glVertex3f(pos[0],pos[1],pos[2]);
		// glVertex3f(world_thruster1(0,3),world_thruster1(1,3),world_thruster1(2,3));

		// glVertex3f(pos[0],pos[1],pos[2]);
		// glVertex3f(world_thruster2(0,3),world_thruster2(1,3),world_thruster2(2,3));

		// glVertex3f(pos[0],pos[1],pos[2]);
		// glVertex3f(world_masscenter(0,3),world_masscenter(1,3),world_masscenter(2,3));

		// for(RigidBody& weight : weights)
		// {
		// 	TransformationMatrix3D world_weight = m_app.m_vehicle.world_vehicle() * weight.parent_body();
		// 	glVertex3f(pos[0],pos[1],pos[2]);
		// 	glVertex3f(world_weight(0,3),world_weight(1,3),world_weight(2,3));
		// }

		// glEnd();
		
		// paintGridCube(
		// 	40, 50, 50, 50, 0, 0, 0
		// );
		 
		
		glPushMatrix();

		cv::Matx44f transposed = coordinateSystem.t();
		glMultMatrixf(&transposed(0,0));

		glPushMatrix();
		glTranslatef(0,0,50);
		glRotatef(90,1,0,0);
		glRotatef(45,0,0,1);
		paintGridCube(
			2, 5, 5, 5, 0, 0, 0
		);
		glPopMatrix();

		// glPushMatrix();
		glRotatef(45,1,0,0);
		
		paintGridSpotlight(
			cv::Vec2f(0,0),
			100,5
		);

		// glPopMatrix();

		glPopMatrix();

		checkGlError();

	}
	void GLWidget::paintGridCube(float interval, int w, int h, int l, float x0, float y0, float z0)
	{
		glBegin(GL_LINES);
			glColor3f(1,1,1);
			for(int x=0; x<w; x++)
			{
				for(int y=0; y<h; y++)
				{
					glVertex3f(
						x0-interval*w/2+x*interval,
						y0-interval*h/2+y*interval,
						z0-interval*l/2+0*interval
					);
					glVertex3f(
						x0-interval*w/2+x*interval,
						y0-interval*h/2+y*interval,
						z0-interval*l/2+(l-1)*interval
					);
				}
			}
			for(int x=0; x<w; x++)
			{
				for(int z=0; z<l; z++)
				{
					glVertex3f(
						x0-interval*w/2+x*interval,
						y0-interval*h/2+0*interval,
						z0-interval*l/2+z*interval
					);
					glVertex3f(
						x0-interval*w/2+x*interval,
						y0-interval*h/2+(h-1)*interval,
						z0-interval*l/2+z*interval
					);
				}
			}
			for(int y=0; y<h; y++)
			{
				for(int z=0; z<l; z++)
				{
					glVertex3f(
						x0-interval*w/2+0*interval,
						y0-interval*h/2+y*interval,
						z0-interval*l/2+z*interval
					);
					glVertex3f(
						x0-interval*w/2+(w-1)*interval,
						y0-interval*h/2+y*interval,
						z0-interval*l/2+z*interval
					);
				}
			}
		glEnd();

	}
	
	void GLWidget::paintAxes(float size)
	{
		glBegin(GL_LINES);
			// x-axis in red
			glColor3f(1,0,0);
			glVertex3f(0,0,0);
			glVertex3f(size,0,0);

			// y-axis in green
			glColor3f(0,1,0);
			glVertex3f(0,0,0);
			glVertex3f(0,size,0);

			// z-axis in blue
			glColor3f(0,0,1);
			glVertex3f(0,0,0);
			glVertex3f(0,0,size);
		glEnd();

	}
	
	void GLWidget::paintAxes(const cv::Matx44f& coordinateSystem, float size)
	{
		glPushMatrix();
		cv::Matx44f transposed = coordinateSystem.t();
		glMultMatrixf(&transposed(0,0));
		paintAxes(size);
		glPopMatrix();
	}

	cv::Vec3f GLWidget::numberedColor(int k)
	{
		float r,g,b;
		int i = ((k%7) % 0b1000)+1;
		r = (i >> 0) & 0x1;
		g = (i >> 1) & 0x1;
		b = (i >> 2) & 0x1;
		return cv::Vec3f(r,g,b);
	}

	void GLWidget::paintGridSpotlight(const cv::Vec2f& xyPosition, float size, float interval)
	{
		int num = (int)ceil(size / interval);
		if (num % 2 == 1)
		{
			num++;
		}
		int half=num/2;
		cv::Vec4f colorCenter(1.0f,1.0f,1.0f,1.0f);
		cv::Vec4f colorBorder(0.0f,0.0f,0.0f,0.0f);
		cv::Mat_<cv::Vec4f> colors(num,num);
		cv::Mat_<cv::Vec3f> vertices(num,num);
		float maxDist = 0.9*sqrt(2*(half*interval)*(half*interval));
		float xOffset = xyPosition[0];
		float yOffset = xyPosition[1];
		// float xOffset = floor(xyPosition[0] / interval) * interval;
		// float yOffset = floor(xyPosition[1] / interval) * interval;
		for(int i=0; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				float x = (i-half)*interval;
				float y = (j-half)*interval;
				float vx = floor((x+xOffset) / interval) * interval;
				float vy = floor((y+yOffset) / interval) * interval;

				float dx = (vx-xOffset);
				float dy = (vy-yOffset);
				float d=sqrt(dx*dx+dy*dy);
				colors(i,j) = colorCenter + (d/maxDist)*(colorBorder-colorCenter);
				
				vertices(i,j) = cv::Vec3f(vx,vy,0);
			}
		}
		glBegin(GL_LINES);
		for(int i=0; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				float x0 = (i+0-half)*interval;
				float x1 = (i+1-half)*interval;
				float y0 = (j+0-half)*interval;
				float y1 = (j+1-half)*interval;
				if(i<num-1)
				{
					glColor4f(colors(i,j)[0],colors(i,j)[1],colors(i,j)[2],colors(i,j)[3]);
					glVertex3f(vertices(i,j)[0],vertices(i,j)[1],vertices(i,j)[2]);
					glColor4f(colors(i+1,j)[0],colors(i+1,j)[1],colors(i+1,j)[2],colors(i+1,j)[3]);
					glVertex3f(vertices(i+1,j)[0],vertices(i+1,j)[1],vertices(i+1,j)[2]);
				}
				if(j<num-1)
				{
					glColor4f(colors(i,j)[0],colors(i,j)[1],colors(i,j)[2],colors(i,j)[3]);
					glVertex3f(vertices(i,j)[0],vertices(i,j)[1],vertices(i,j)[2]);
					glColor4f(colors(i,j+1)[0],colors(i,j+1)[1],colors(i,j+1)[2],colors(i,j+1)[3]);
					glVertex3f(vertices(i,j+1)[0],vertices(i,j+1)[1],vertices(i,j+1)[2]);
				}
			}
		}
		glEnd();
	}

	void GLWidget::checkGlError()
	{
		int err = glGetError();
		if(err == GL_NO_ERROR)
		{
			// cout << "ok" << endl;
			return;
		}
		else
		{
			cout << "err "  << err << endl;
			switch(err)
			{
				case GL_NO_ERROR:
					cout << "GL_NO_ERROR" << endl;
					break;
				case GL_INVALID_ENUM:
					cout << "GL_INVALID_ENUM" << endl;
					break;
				case GL_INVALID_VALUE:
					cout << "GL_INVALID_VALUE" << endl;
					break;
				case GL_INVALID_OPERATION:
					cout << "GL_INVALID_OPERATION" << endl;
					break;
				case GL_INVALID_FRAMEBUFFER_OPERATION:
					cout << "GL_INVALID_FRAMEBUFFER_OPERATION" << endl;
					break;
				case GL_OUT_OF_MEMORY:
					cout << "GL_OUT_OF_MEMORY" << endl;
					break;
				case GL_STACK_UNDERFLOW:
					cout << "GL_STACK_UNDERFLOW" << endl;
					break;
				case GL_STACK_OVERFLOW:
					cout << "GL_STACK_OVERFLOW" << endl;
					break;
			}
		}
	}

	void GLWidget::resizeGL(int width, int height)
	{
		glViewport(0, 0, width, height);
		m_app.m_camera->setProjection(width, height);
	}

	void GLWidget::mousePressEvent(QMouseEvent* event)
	{
		m_app.m_camera->mousePressEvent(event);
	}

	void GLWidget::mouseReleaseEvent(QMouseEvent* event)
	{
		m_app.m_camera->mouseReleaseEvent(event);
	}

	void GLWidget::mouseMoveEvent(QMouseEvent* event)
	{
		m_app.m_camera->mouseMoveEvent(event);
		update();
	}

	void GLWidget::wheelEvent(QWheelEvent* event)
	{
		m_app.m_camera->wheelEvent(event);
	}

	void GLWidget::keyPressEvent(QKeyEvent* event)
	{
		m_app.m_keyboard->keyPressEvent(event);
	}

	void GLWidget::keyReleaseEvent(QKeyEvent* event)
	{
		m_app.m_keyboard->keyReleaseEvent(event);
	}


} // namespace Gui
} // namespace ar
