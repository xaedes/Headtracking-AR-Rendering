#include <iostream>

#include "TEMPLATE/version.h"

using namespace ar::TEMPLATE;

int main(int argc, char* argv[])
{	
    std::cout << "version " << Version::getString() << std::endl;
    std::cout << "revision " << Version::getRevision() << std::endl;
}
