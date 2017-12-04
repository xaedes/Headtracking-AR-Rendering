#pragma once
#include <string>
#include <vector>

namespace ar {
namespace common {

    std::vector<std::string> get_arguments(int argc, char **argv)
    {
        std::vector<std::string> arguments;

        // First argument is reserved for the name of the executable
        for(int i = 0; i < argc; ++i)
        {
            arguments.push_back(std::string(argv[i]));
        }
        return arguments;
    }

} // namespace common
} // namespace ar
