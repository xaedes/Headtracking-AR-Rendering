cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../cmake) # main (top) cmake dir
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake) # project specific cmake dir
set(CMAKE_CXX_STANDARD 11) # tODO move up to a general cmake config for all sub projects ?

# CMake useful variables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Set the name of your project here
project("FaceDetection")

# Set the version number of your project here (format is MAJOR.MINOR.PATCHLEVEL - e.g. 1.0.0)
set(VERSION_MAJOR "1")
set(VERSION_MINOR "0")
set(VERSION_PATCH "0")
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

include(Common)

find_package(OpenCV)
find_package(OpenNI2)

find_package(OpenFace)
message("OpenFace_LIBRARIES\t${OpenFace_LIBRARIES}")
message("OpenFace_CONFIG_DIR\t${OpenFace_CONFIG_DIR}")

# dlib

# dlib library
add_subdirectory(external/dlib)

find_package( BLAS REQUIRED )
# include_directories( ${BLAS_INCLUDE_DIRS} )
link_directories(${BLAS_LIBRARY_DIRS})  

find_package( Boost 1.5.9 REQUIRED COMPONENTS filesystem system)
# message("Boost information:") 
# message("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
# message("  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
# message("  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}") 

link_directories(${Boost_LIBRARY_DIRS}) 

find_package(TBB REQUIRED)


## Git (and its revision)
find_package(Git QUIET) # if we don't find git or FindGit.cmake is not on the system we ignore it.
## GetGitRevisionDescription module to retreive branch and revision information from Git
## Starting with Git 1.9 the module will be part of official cMake distribution, until then it has to be
## part of the application
## The Git module will trigger a reconfiguration for each pull that will bring a new revision on the local repository
set (VCS_REVISION "-1")
if(GIT_FOUND)
    include(GetGitRevisionDescription)
    get_git_head_revision(GIT_REFSPEC GIT_SHA1)
    # message(STATUS "GIT branch ${GIT_REFSPEC}")
    # message(STATUS "GIT revision ${GIT_SHA1}")
    set (VCS_REVISION ${GIT_SHA1})
endif()

# This sets the include directory for the reference project. This is the -I flag in gcc.
# All the includes should be in this variable To add new directories to the include path,
# just add them in this list (after a space), or add an include_directories statement in a
# subdirectory's CMakeLists.
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_BINARY_DIR}/include   # add the binary tree to the search path so that we will find version.h
)

enable_testing()
add_subdirectory(include)
add_subdirectory(src)



# add_subdirectory(doc)
# add_subdirectory(test)

# 
# SYSTEM PACKAGING (RPM, TGZ, ...)
# _____________________________________________________________________________

# include(CPackConfig)

# 
# CMAKE PACKAGING (for other CMake projects to use this one easily)
# _____________________________________________________________________________

# include(PackageConfigurator)