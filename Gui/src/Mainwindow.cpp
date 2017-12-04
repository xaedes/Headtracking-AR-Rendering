#include <QStringList>
#include <iostream>

#include "Mainwindow.h"
#include "ui_Mainwindow.h"
#include "App.h"

using namespace std;
// ‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗‗

namespace ar {
namespace Gui {

	MainWindow::MainWindow(App& app, QWidget *parent) :
		QMainWindow(parent),
		m_app(app),
		m_ui(new Ui::MainWindow),
		m_glWidget(new GLWidget(m_app))
	{
		m_ui->setupUi(this);
		//m_glWidget = new GLWidget(m_app);
		m_ui->container->addWidget(m_glWidget.get());

	}

	QStatusBar* MainWindow::getStatusBar()
	{
		return m_ui->statusbar;
	}

	MainWindow::~MainWindow()
	{
	}

} // namespace Gui
} // namespace ar
