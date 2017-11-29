#include <iostream>

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

int main(int argc, char* argv[])
{	
    std::cout << "version " << Version::getString() << std::endl;
    std::cout << "revision " << Version::getRevision() << std::endl;


    AsusXtionSensor sensor;
    Streamer_<cv::Mat_<cv::Vec3b>> color = sensor.colorStream();
    Streamer_<cv::Mat_<uint16_t>> depth = sensor.depthStream();

    int key = 255;
    cv::Mat_<cv::Vec3b> colorFrame;
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
    LandmarkDetector::FaceModelParameters det_parameters;
    // Always track gaze in feature extraction
    det_parameters.track_gaze = true;
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

        frame_count += 1;
        {

            // Reading the images
            cv::Mat_<uchar> grayscale_image;

            if(colorFrame.channels() == 3)
            {
                cvtColor(colorFrame, grayscale_image, CV_BGR2GRAY);             
            }
            else
            {
                grayscale_image = colorFrame.clone();               
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
                cx = colorFrame.cols / 2.0f;
                cy = colorFrame.rows / 2.0f;
            }
            // Use a rough guess-timate of focal length
            // if (fx_undefined)
            {
                fx = 500 * (colorFrame.cols / 640.0);
                fy = 500 * (colorFrame.rows / 480.0);

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
            visualise_tracking(colorFrame, face_model, det_parameters, gazeDirection0, gazeDirection1, frame_count, fx, fy, cx, cy);

        }

        if(colorFrame.data)
        {
            cv::imshow("color", colorFrame);
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
