#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include "FaceDetection/version.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "FaceDetection/FaceDetection.h"
// #include <OpenFace/LandmarkDetector/LandmarkCoreIncludes.h>
// #include <OpenFace/FaceAnalyser/Face_utils.h>
// #include <OpenFace/FaceAnalyser/FaceAnalyser.h>
// #include <OpenFace/GazeAnalyser/GazeEstimation.h>

#include "AsusXtionSensor/AsusXtionSensor.h"

using namespace ar::FaceDetection;
using namespace ar::AsusXtionSensor;


// std::ostream& operator<<(std::ostream& os, const std::vector<int>& vecInt)
// {
//     os << "[";
//     for(int i=0; i < vecInt.size()-1; i++)
//     {
//         os << vecInt[i] << ", ";
//     }
//     os << vecInt[vecInt.size()-1];
//     os << "]";
//     return os;
// }

// std::ostream& operator<<(std::ostream& os, const LandmarkDetector::FaceModelParameters::FaceDetector& faceDetector)
// {
//     switch(faceDetector)
//     {
//     case LandmarkDetector::FaceModelParameters::FaceDetector::HAAR_DETECTOR:
//         os << "HAAR_DETECTOR";
//         break;
//     case LandmarkDetector::FaceModelParameters::FaceDetector::HOG_SVM_DETECTOR:
//         os << "HOG_SVM_DETECTOR";
//         break;
//     default:
//         os << static_cast<int>(faceDetector);
//     }
//     return os;
// }

// std::ostream& operator<<(std::ostream& os, const LandmarkDetector::FaceModelParameters& parameters)
// {
//     os << "num_optimisation_iteration\t" << parameters.num_optimisation_iteration << std::endl;
//     os << "limit_pose\t" << parameters.limit_pose << std::endl;
//     os << "validate_detections\t" << parameters.validate_detections << std::endl;
//     os << "validation_boundary\t" << parameters.validation_boundary << std::endl;
//     os << "window_sizes_small\t" << parameters.window_sizes_small << std::endl;
//     os << "window_sizes_init\t" << parameters.window_sizes_init << std::endl;
//     os << "window_sizes_current\t" << parameters.window_sizes_current << std::endl;
//     os << "face_template_scale\t" << parameters.face_template_scale << std::endl;
//     os << "use_face_template\t" << parameters.use_face_template << std::endl;
//     os << "model_location\t" << parameters.model_location << std::endl;
//     os << "sigma\t" << parameters.sigma << std::endl;
//     os << "reg_factor\t" << parameters.reg_factor << std::endl;
//     os << "weight_factor\t" << parameters.weight_factor << std::endl;
//     os << "multi_view\t" << parameters.multi_view << std::endl;
//     os << "reinit_video_every\t" << parameters.reinit_video_every << std::endl;
//     os << "face_detector_location\t" << parameters.face_detector_location << std::endl;
//     os << "curr_face_detector\t" << parameters.curr_face_detector << std::endl;
//     os << "quiet_mode\t" << parameters.quiet_mode << std::endl;
//     os << "refine_hierarchical\t" << parameters.refine_hierarchical << std::endl;
//     os << "refine_parameters\t" << parameters.refine_parameters << std::endl;
//     os << "track_gaze\t" << parameters.track_gaze << std::endl;
//     return os;
// }

// std::ostream& operator<<(std::ostream& os, const MyParameters& myParameters)
// {
//     os << "scale\t" << myParameters.scale << std::endl;
//     return os;
// }

int main(int argc, char* argv[])
{	
    std::cout << "version " << Version::getString() << std::endl;
    std::cout << "revision " << Version::getRevision() << std::endl;


    AsusXtionSensor sensor;
    Streamer_<cv::Mat_<cv::Vec3b>> color = sensor.colorStream();
    Streamer_<cv::Mat_<uint16_t>> depth = sensor.depthStream();

    FaceDetection faceDetection(argc, argv);

    int key = 255;
    cv::Mat_<cv::Vec3b> colorFrame;
    cv::Mat_<uint16_t> depthFrame;
    cv::Mat visualizedFrame;


    // std::cout << "det_parameters" << std::endl;
    // std::cout << "----" << std::endl;
    // std::cout << det_parameters << std::endl;
    // std::cout << "----" << std::endl;
    // std::cout << std::endl;

    // std::cout << "myParameters" << std::endl;
    // std::cout << "----" << std::endl;
    // std::cout << myParameters << std::endl;
    // std::cout << "----" << std::endl;
    // std::cout << std::endl;

    // -face_detector haar
    // det_parameters.curr_face_detector = HAAR_DETECTOR;
    // -n_iter 2
    // det_parameters.num_optimisation_iteration = 2;
    // -refine_hierarchical 0
    // det_parameters.refine_hierarchical = false;
    // -refine_parameters 0
    // det_parameters.refine_parameters = false;
    
    // -reinit_video_every 20

    // geht nich:
    // -validate_detections 0
    // det_parameters.validate_detections = false;

    std::string csv_filename = "results.csv";
    std::ofstream csv_file;
    csv_file.open(csv_filename, std::ios::out | std::ios::app);
    csv_file   << "\"time\"" 
        << ";" << "\"detectionCertainty\""
        << ";" << "\"x\""
        << ";" << "\"y\"" 
        << ";" << "\"z\"" 
        << ";" << "\"euler0\"" 
        << ";" << "\"euler1\"" 
        << ";" << "\"euler2\""          
        << std::endl;

    csv_file 
        << std::fixed
        << std::setprecision(3)
        ;
    csv_file.close();

    
    std::cout << "key\t" << (key & 0xff) << std::endl;
    while(1)
    {
        color >> colorFrame;
        depth >> depthFrame;
        faceDetection << colorFrame;
        faceDetection >> visualizedFrame;

        cv::Vec6d headPose = faceDetection.headPose();


        csv_file.open(csv_filename, std::ios::out | std::ios::app);
        csv_file 
            << std::fixed
            << std::setprecision(3);
        csv_file
            << cv::getTickCount()
            << ";" << faceDetection.detectionCertainty()
            << ";" << headPose[0]
            << ";" << headPose[1]
            << ";" << headPose[2]
            << ";" << headPose[3]
            << ";" << headPose[4]
            << ";" << headPose[5]
            << std::endl;
        csv_file.close();

        if(colorFrame.data)
        {
            cv::imshow("color", colorFrame);
        }
        if(visualizedFrame.data)
        {
            cv::imshow("visualized", visualizedFrame);
        }
        if(depthFrame.data)
        {
            cv::Mat_<float> depthFloat(depthFrame);
            depthFloat = 1-(depthFloat / (float)0xffff);
            cv::imshow("depth", depthFloat);
        }
        key = cv::waitKey(10);
        // std::cout << "key\t" << (key & 0xff) << std::endl;
    }

}
