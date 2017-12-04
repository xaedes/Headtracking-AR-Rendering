#include <iostream>
#include "Gui/version.h"
#include "App.h"

using namespace ar::Gui;

int main(int argc, char* argv[])
{
	std::cout << "version " << Version::getString() << std::endl;
	std::cout << "revision " << Version::getRevision() << std::endl;

	App app;
	return app.run(argc,argv);
}
