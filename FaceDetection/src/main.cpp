#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include "FaceDetection/version.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <OpenFace/LandmarkDetector/LandmarkCoreIncludes.h>
#include <OpenFace/FaceAnalyser/Face_utils.h>
#include <OpenFace/FaceAnalyser/FaceAnalyser.h>
#include <OpenFace/GazeAnalyser/GazeEstimation.h>

#include "AsusXtionSensor/AsusXtionSensor.h"

using namespace ar::FaceDetection;
using namespace ar::AsusXtionSensor;

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

// Visualising the results
void visualise_tracking(cv::Mat& captured_image, const LandmarkDetector::CLNF& face_model, const LandmarkDetector::FaceModelParameters& det_parameters, cv::Point3f gazeDirection0, cv::Point3f gazeDirection1, int frame_count, double fx, double fy, double cx, double cy)
{
    // Some globals for tracking timing information for visualisation
    static double fps_tracker = -1.0;
    static int64 t0 = 0;


    // Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
    double detection_certainty = face_model.detection_certainty;
    bool detection_success = face_model.detection_success;

    double visualisation_boundary = 0.2;

    // Only draw if the reliability is reasonable, the value is slightly ad-hoc
    if (detection_certainty < visualisation_boundary)
    {
        LandmarkDetector::Draw(captured_image, face_model);

        double vis_certainty = detection_certainty;
        if (vis_certainty > 1)
            vis_certainty = 1;
        if (vis_certainty < -1)
            vis_certainty = -1;

        vis_certainty = (vis_certainty + 1) / (visualisation_boundary + 1);

        // A rough heuristic for box around the face width
        int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

        cv::Vec6d pose_estimate_to_draw = LandmarkDetector::GetPose(face_model, fx, fy, cx, cy);

        // Draw it in reddish if uncertain, blueish if certain
        LandmarkDetector::DrawBox(captured_image, pose_estimate_to_draw, cv::Scalar((1 - vis_certainty)*255.0, 0, vis_certainty * 255), thickness, fx, fy, cx, cy);

        if (det_parameters.track_gaze && detection_success && face_model.eye_model)
        {
            GazeAnalysis::DrawGaze(captured_image, face_model, gazeDirection0, gazeDirection1, fx, fy, cx, cy);
        }
    }

    // Work out the framerate
    if (frame_count % 10 == 0)
    {
        double t1 = cv::getTickCount();
        fps_tracker = 10.0 / (double(t1 - t0) / cv::getTickFrequency());
        t0 = t1;
    }

    // Write out the framerate on the image before displaying it
    char fpsC[255];
    std::sprintf(fpsC, "%d", (int)fps_tracker);
    string fpsSt("FPS:");
    fpsSt += fpsC;
    cv::putText(captured_image, fpsSt, cv::Point(10, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1, CV_AA);

    if (!det_parameters.quiet_mode)
    {
        cv::namedWindow("tracking_result", 1);
        cv::imshow("tracking_result", captured_image);
    }
}

struct MyParameters
{
    float scale;
};

void applyOwnArguments(LandmarkDetector::FaceModelParameters& parameters, MyParameters& myParameters, std::vector<std::string>& arguments)
{
    std::vector<bool> consumeArgument(arguments.size());
    for(int i=0; i < arguments.size(); i++)
    {
        consumeArgument.push_back(false);
    }

    for (size_t i = 1; i < arguments.size(); ++i)
    {
        consumeArgument[i] = false;

        if (arguments[i].compare("-face_detector") == 0)
        {
            consumeArgument[i] = true;
            if (i+1 < arguments.size())
            {
                std::string faceDetector = arguments[i + 1];
                bool validFaceDetector = false;
                if (faceDetector.compare("haar") == 0)
                {

                    parameters.curr_face_detector = LandmarkDetector::FaceModelParameters::HAAR_DETECTOR;
                    validFaceDetector = true;
                }
                else if (faceDetector.compare("svm") == 0)
                {
                    parameters.curr_face_detector = LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR;
                    validFaceDetector = true;
                }
                consumeArgument[i + 1] = true;
                i++;
            }
        }
        else if (arguments[i].compare("-refine_hierarchical") == 0)
        {
            consumeArgument[i] = true;
            if (i+1 < arguments.size())
            {
                std::stringstream sstream(arguments[i + 1]);
                int value;
                sstream >> value;

                parameters.refine_hierarchical = (bool)(value != 0);

                consumeArgument[i + 1] = true;
                i++;
            }
        }
        else if (arguments[i].compare("-refine_parameters") == 0)
        {
            consumeArgument[i] = true;
            if (i+1 < arguments.size())
            {
                std::stringstream sstream(arguments[i + 1]);
                int value;
                sstream >> value;

                parameters.refine_parameters = (bool)(value != 0);

                consumeArgument[i + 1] = true;
                i++;
            }
        }
        else if (arguments[i].compare("-reinit_video_every") == 0)
        {
            consumeArgument[i] = true;
            if (i+1 < arguments.size())
            {
                std::stringstream sstream(arguments[i + 1]);
                int value;
                sstream >> value;

                parameters.reinit_video_every = value;

                consumeArgument[i + 1] = true;
                i++;
            }
        }
        else if (arguments[i].compare("-scale") == 0)
        {
            consumeArgument[i] = true;
            if (i+1 < arguments.size())
            {
                std::stringstream sstream(arguments[i + 1]);
                float value;
                sstream >> value;

                myParameters.scale = value;

                consumeArgument[i + 1] = true;
                i++;
            }
        }
    }
    for (int i = (int)arguments.size() - 1; i >= 0; --i)
    {
        if (consumeArgument[i])
        {
            arguments.erase(arguments.begin() + i);
        }
    }
}

std::ostream& operator<<(std::ostream& os, const std::vector<int>& vecInt)
{
    os << "[";
    for(int i=0; i < vecInt.size()-1; i++)
    {
        os << vecInt[i] << ", ";
    }
    os << vecInt[vecInt.size()-1];
    os << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const LandmarkDetector::FaceModelParameters::FaceDetector& faceDetector)
{
    switch(faceDetector)
    {
    case LandmarkDetector::FaceModelParameters::FaceDetector::HAAR_DETECTOR:
        os << "HAAR_DETECTOR";
        break;
    case LandmarkDetector::FaceModelParameters::FaceDetector::HOG_SVM_DETECTOR:
        os << "HOG_SVM_DETECTOR";
        break;
    default:
        os << static_cast<int>(faceDetector);
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const LandmarkDetector::FaceModelParameters& parameters)
{
    os << "num_optimisation_iteration\t" << parameters.num_optimisation_iteration << std::endl;
    os << "limit_pose\t" << parameters.limit_pose << std::endl;
    os << "validate_detections\t" << parameters.validate_detections << std::endl;
    os << "validation_boundary\t" << parameters.validation_boundary << std::endl;
    os << "window_sizes_small\t" << parameters.window_sizes_small << std::endl;
    os << "window_sizes_init\t" << parameters.window_sizes_init << std::endl;
    os << "window_sizes_current\t" << parameters.window_sizes_current << std::endl;
    os << "face_template_scale\t" << parameters.face_template_scale << std::endl;
    os << "use_face_template\t" << parameters.use_face_template << std::endl;
    os << "model_location\t" << parameters.model_location << std::endl;
    os << "sigma\t" << parameters.sigma << std::endl;
    os << "reg_factor\t" << parameters.reg_factor << std::endl;
    os << "weight_factor\t" << parameters.weight_factor << std::endl;
    os << "multi_view\t" << parameters.multi_view << std::endl;
    os << "reinit_video_every\t" << parameters.reinit_video_every << std::endl;
    os << "face_detector_location\t" << parameters.face_detector_location << std::endl;
    os << "curr_face_detector\t" << parameters.curr_face_detector << std::endl;
    os << "quiet_mode\t" << parameters.quiet_mode << std::endl;
    os << "refine_hierarchical\t" << parameters.refine_hierarchical << std::endl;
    os << "refine_parameters\t" << parameters.refine_parameters << std::endl;
    os << "track_gaze\t" << parameters.track_gaze << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const MyParameters& myParameters)
{
    os << "scale\t" << myParameters.scale << std::endl;
    return os;
}

int main(int argc, char* argv[])
{	
    std::cout << "version " << Version::getString() << std::endl;
    std::cout << "revision " << Version::getRevision() << std::endl;

    std::vector<std::string> arguments = get_arguments(argc, argv);

    AsusXtionSensor sensor;
    Streamer_<cv::Mat_<cv::Vec3b>> color = sensor.colorStream();
    Streamer_<cv::Mat_<uint16_t>> depth = sensor.depthStream();

    int key = 255;
    cv::Mat_<cv::Vec3b> colorFrame;
    cv::Mat_<cv::Vec3b> colorFrameScaled;
    cv::Mat_<uint16_t> depthFrame;

    bool output_2D_landmarks = true;
    bool output_3D_landmarks = true;
    bool output_model_params = true;
    bool output_pose = true;
    bool output_AUs = true;
    bool output_gaze = true;

    bool visualize_track = false;
    bool visualize_align = false;
    bool visualize_hog = false;
    // Load the modules that are being used for tracking and face analysis
    // Load face landmark detector
    LandmarkDetector::FaceModelParameters det_parameters(arguments);
    MyParameters myParameters{1.0};
    applyOwnArguments(det_parameters, myParameters, arguments);
    // Always track gaze in feature extraction
    det_parameters.track_gaze = true;

    std::cout << "det_parameters" << std::endl;
    std::cout << "----" << std::endl;
    std::cout << det_parameters << std::endl;
    std::cout << "----" << std::endl;
    std::cout << std::endl;

    std::cout << "myParameters" << std::endl;
    std::cout << "----" << std::endl;
    std::cout << myParameters << std::endl;
    std::cout << "----" << std::endl;
    std::cout << std::endl;

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
    
    LandmarkDetector::CLNF face_model(det_parameters.model_location);

    // Load facial feature extractor and AU analyser
    FaceAnalysis::FaceAnalyserParameters face_analysis_params;
    FaceAnalysis::FaceAnalyser face_analyser(face_analysis_params);

    std::cout << "key\t" << (key & 0xff) << std::endl;
    int frame_count = 0;
    while(1)
    {
        color >> colorFrame;
        depth >> depthFrame;

        if(myParameters.scale != 1.0 && 0 < myParameters.scale && myParameters.scale < 16)
        {
            cv::Size size = colorFrame.size();
            cv::Size scaledSize((int)ceil(size.width * myParameters.scale), (int)ceil(size.height * myParameters.scale));
            cv::resize(colorFrame, colorFrameScaled, scaledSize);
        }
        else
        {
            colorFrameScaled = colorFrame;               
        }

        frame_count += 1;
        {

            // Reading the images
            cv::Mat_<uchar> grayscale_image;

            if(colorFrameScaled.channels() == 3)
            {
                cvtColor(colorFrameScaled, grayscale_image, CV_BGR2GRAY);             
            }
            else
            {
                grayscale_image = colorFrameScaled.clone();               
            }


            // The actual facial landmark detection / tracking
            bool detection_success;
            
            detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, face_model, det_parameters);

            // Gaze tracking, absolute gaze direction
            cv::Point3f gazeDirection0(0, 0, -1);
            cv::Point3f gazeDirection1(0, 0, -1);
            cv::Vec2d gazeAngle(0, 0);

            float fx = 0, fy = 0, cx = 0, cy = 0;
            // If optical centers are not defined just use center of image
            // if(cx_undefined)
            {
                cx = colorFrameScaled.cols / 2.0f;
                cy = colorFrameScaled.rows / 2.0f;
            }
            // Use a rough guess-timate of focal length
            // if (fx_undefined)
            {
                fx = 500 * (colorFrameScaled.cols / 640.0);
                fy = 500 * (colorFrameScaled.rows / 480.0);

                fx = (fx + fy) / 2.0;
                fy = fx;
            }

            if (det_parameters.track_gaze && detection_success && face_model.eye_model)
            {
                GazeAnalysis::EstimateGaze(face_model, gazeDirection0, fx, fy, cx, cy, true);
                GazeAnalysis::EstimateGaze(face_model, gazeDirection1, fx, fy, cx, cy, false);
                gazeAngle = GazeAnalysis::GetGazeAngle(gazeDirection0, gazeDirection1);
            }

            // Work out the pose of the head from the tracked model
            cv::Vec6d pose_estimate = LandmarkDetector::GetPose(face_model, fx, fy, cx, cy);

            // Visualising the tracker
            visualise_tracking(colorFrameScaled, face_model, det_parameters, gazeDirection0, gazeDirection1, frame_count, fx, fy, cx, cy);

        }

        if(colorFrameScaled.data)
        {
            cv::imshow("color", colorFrameScaled);
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
