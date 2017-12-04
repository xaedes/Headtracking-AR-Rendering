#pragma once

#include <QApplication>
#include <QObject>
#include <memory>
#include <unordered_map>
#include <string>
#include <opencv2/core.hpp>

#include "AsusXtionSensor/AsusXtionSensor.h"
#include "FaceDetectionComponent/FaceDetectionComponent.h"

namespace ar {
namespace Gui {

	// forward declarations
	class MainWindow;
	class Keyboard;
	class KeyboardControl;
	class Camera;
	
	/// \brief      Holds all the gui application data.
	class App : public QObject
	{
		Q_OBJECT
		
	public:
		App();
		~App();
		int run(int argc, char* argv[]);
		
		std::unique_ptr<QApplication> m_qApp;
		std::unique_ptr<MainWindow> m_mainWindow;
		std::unique_ptr<Camera> m_camera;
		std::unique_ptr<Keyboard> m_keyboard;
		std::unique_ptr<KeyboardControl> m_keyboardControl;

	    AsusXtionSensor::AsusXtionSensor m_sensor;
    	AsusXtionSensor::Streamer_<cv::Mat_<cv::Vec3b>> m_colorStream;
    	FaceDetectionComponent::FaceDetectionComponent m_faceDetection;

		void showStatusMessage(std::string msg);

	};

} // namespace Gui
} // namespace ar
