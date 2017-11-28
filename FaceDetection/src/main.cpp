#include <iostream>

#include "FaceDetection/version.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <OpenFace/LandmarkDetector/LandmarkCoreIncludes.h>
#include <OpenFace/FaceAnalyser/Face_utils.h>
#include <OpenFace/FaceAnalyser/FaceAnalyser.h>
#include <OpenFace/GazeAnalyser/GazeEstimation.h>

using namespace ar::FaceDetection;

int main(int argc, char* argv[])
{	
    std::cout << "version " << Version::getString() << std::endl;
    std::cout << "revision " << Version::getRevision() << std::endl;
}
