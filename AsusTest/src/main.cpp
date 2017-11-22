#include <iostream>

#include "AsusTest/version.h"

using namespace ar::AsusTest;

int main(int argc, char* argv[])
{	
    std::cout << "version " << Version::getString() << std::endl;
    std::cout << "revision " << Version::getRevision() << std::endl;
}
