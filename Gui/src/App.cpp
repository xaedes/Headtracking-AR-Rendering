#include "App.h"
#include "Mainwindow.h"
#include "Keyboard.h"
#include "KeyboardControl.h"
#include "Camera.h"

#include <iostream>

using namespace std;

namespace ar {
namespace Gui {

	App::App()
	{
	}

	App::~App()
	{
	}

	int App::run(int argc, char* argv[])
	{
    	m_faceDetection.start(argc,argv);
    	m_colorStream = m_sensor.colorStream();

		m_keyboard.reset(new Keyboard());
		m_camera.reset(new Camera(*m_keyboard.get()));
		
		m_keyboardControl.reset(new KeyboardControl(*this));

		m_qApp.reset(new QApplication(argc, &argv[0]));
		m_mainWindow.reset(new MainWindow(*this));
		m_mainWindow->show();

		return m_qApp->exec();
	}

	void App::showStatusMessage(std::string msg)
	{
		if (m_mainWindow.get())
		{
			QStatusBar *bar=m_mainWindow->getStatusBar();
			if (bar)
			{
				bar->showMessage(QString::fromStdString(msg));
			}
		}
	}

} // namespace Gui
} // namespace ar
