#pragma once

#include <QMainWindow>
#include <QFileSystemModel>
#include <QItemSelection>
#include <QStatusBar>
#include <QStringListModel>

#include <memory>

#include "GLWidget.h"

// forward declare Qt generated class
namespace Ui {
class MainWindow;
}

namespace ar {
namespace Gui {

	// forward declaration
	class App;

	class MainWindow : public QMainWindow
	{
		Q_OBJECT

	public:
		explicit MainWindow(App& app, QWidget *parent = 0);
		~MainWindow();

		QStatusBar* getStatusBar();

	public slots:
	private:
		App& m_app;
		std::shared_ptr<Ui::MainWindow> m_ui;
		std::shared_ptr<GLWidget> m_glWidget;
	};

} // namespace Gui
} // namespace ar
