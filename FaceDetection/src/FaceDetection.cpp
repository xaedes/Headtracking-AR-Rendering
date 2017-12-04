#include "FaceDetection/FaceDetection.h"
#include <cstring>

#include "common/get_arguments.h"

using ar::common::get_arguments;

namespace ar {
namespace FaceDetection {



    FaceDetectionAdditionalParameters::FaceDetectionAdditionalParameters()
    {
        scale = 1.0;
        visualization = false;
    }

    FaceDetection::FaceDetection()
    {
        m_faceModelParameters = LandmarkDetector::FaceModelParameters();
        m_additionalParameters = FaceDetectionAdditionalParameters();
        m_faceModelParameters.track_gaze = true;
        init();
    }

    FaceDetection::FaceDetection(int argc, char **argv)
    {
        std::vector<std::string> arguments = get_arguments(argc, argv);
        m_faceModelParameters = LandmarkDetector::FaceModelParameters(arguments);
        m_additionalParameters = FaceDetectionAdditionalParameters();
        applyArguments(m_faceModelParameters, m_additionalParameters, arguments);
        init();
    }
    void FaceDetection::init()
    {
        m_fpsTracker = -1.0;
        m_t0 = 0;
        m_frameCount = 0;
        m_faceModel.reset(new LandmarkDetector::CLNF(m_faceModelParameters.model_location));
        // m_faceAnalysisParameters.reset(new FaceAnalysis::FaceAnalyserParameters());
        // m_faceAnalyser.reset(new FaceAnalysis::FaceAnalyser(*m_faceAnalysisParameters));
    }

    void FaceDetection::applyArguments(
        LandmarkDetector::FaceModelParameters& parameters, 
        FaceDetectionAdditionalParameters& addParameters, 
        std::vector<std::string>& arguments
    ) const 
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
            else if (arguments[i].compare("-track_gaze") == 0)
            {
                consumeArgument[i] = true;
                if (i+1 < arguments.size())
                {
                    std::stringstream sstream(arguments[i + 1]);
                    int value;
                    sstream >> value;

                    parameters.track_gaze = (bool)(value != 0);

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

                    addParameters.scale = value;

                    consumeArgument[i + 1] = true;
                    i++;
                }
            }
            else if (arguments[i].compare("-visualization") == 0)
            {
                consumeArgument[i] = true;
                if (i+1 < arguments.size())
                {
                    std::stringstream sstream(arguments[i + 1]);
                    int value;
                    sstream >> value;

                    addParameters.visualization = (bool)(value != 0);

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

    FaceDetection::~FaceDetection()
    {}

    void FaceDetection::feedColorFrame(const cv::Mat_<cv::Vec3b>& colorFrame)
    {
        cv::Mat_<cv::Vec3b> colorFrameScaled;
        if(m_additionalParameters.scale != 1.0 && 0 < m_additionalParameters.scale && m_additionalParameters.scale < 16)
        {
            cv::Size size = colorFrame.size();
            cv::Size scaledSize((int)ceil(size.width * m_additionalParameters.scale), (int)ceil(size.height * m_additionalParameters.scale));
            cv::resize(colorFrame, colorFrameScaled, scaledSize);
        }
        else
        {
            colorFrameScaled = colorFrame;               
        }
        
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
        
        detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, *m_faceModel, m_faceModelParameters);

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

        if (m_faceModelParameters.track_gaze && detection_success && m_faceModel->eye_model)
        {
            GazeAnalysis::EstimateGaze(*m_faceModel, gazeDirection0, fx, fy, cx, cy, true);
            GazeAnalysis::EstimateGaze(*m_faceModel, gazeDirection1, fx, fy, cx, cy, false);
            gazeAngle = GazeAnalysis::GetGazeAngle(gazeDirection0, gazeDirection1);
        }

       // Work out the pose of the head from the tracked model
        m_headPose = LandmarkDetector::GetPose(*m_faceModel, fx, fy, cx, cy);

        if(m_additionalParameters.visualization)
        {
            // Visualising the tracker
            visualizeTracking(colorFrameScaled, gazeDirection0, gazeDirection1, fx, fy, cx, cy);
        }

        m_frameCount++;
    }

    FaceDetection& operator<<(FaceDetection& faceDetection, const cv::Mat_<cv::Vec3b>& colorFrame)
    {
        faceDetection.feedColorFrame(colorFrame);
        return faceDetection;
    }

    FaceDetection& operator>>(FaceDetection& faceDetection, cv::Mat& visualizedFrame)
    {
        visualizedFrame = faceDetection.visualization();
        return faceDetection;
    }

    cv::Mat& FaceDetection::visualization()
    {
        return m_visualization;   
    }
    cv::Vec6d FaceDetection::headPose()
    {
        return m_headPose;
    }
    double FaceDetection::detectionCertainty()
    {
        return m_faceModel->detection_certainty;
    }

    void FaceDetection::visualizeTracking(
        const cv::Mat_<cv::Vec3b>& colorFrame, 
        cv::Point3f gazeDirection0, 
        cv::Point3f gazeDirection1, 
        double fx, 
        double fy, 
        double cx, 
        double cy
    ) 
    {
        m_visualization = colorFrame.clone();
        // Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
        double detection_certainty = m_faceModel->detection_certainty;
        bool detection_success = m_faceModel->detection_success;

        double visualisation_boundary = 0.2;

        // Only draw if the reliability is reasonable, the value is slightly ad-hoc
        if (detection_certainty < visualisation_boundary)
        {
            LandmarkDetector::Draw(m_visualization, *m_faceModel);

            double vis_certainty = detection_certainty;
            if (vis_certainty > 1)
                vis_certainty = 1;
            if (vis_certainty < -1)
                vis_certainty = -1;

            vis_certainty = (vis_certainty + 1) / (visualisation_boundary + 1);

            // A rough heuristic for box around the face width
            int thickness = (int)std::ceil(2.0* ((double)m_visualization.cols) / 640.0);

            // Draw it in reddish if uncertain, blueish if certain
            LandmarkDetector::DrawBox(m_visualization, m_headPose, cv::Scalar((1 - vis_certainty)*255.0, 0, vis_certainty * 255), thickness, fx, fy, cx, cy);

            if (m_faceModelParameters.track_gaze && detection_success && m_faceModel->eye_model)
            {
                GazeAnalysis::DrawGaze(m_visualization, *m_faceModel, gazeDirection0, gazeDirection1, fx, fy, cx, cy);
            }
        }

        // Work out the framerate
        if (m_frameCount % 10 == 0)
        {
            double t1 = cv::getTickCount();
            m_fpsTracker = 10.0 / (double(t1 - m_t0) / cv::getTickFrequency());
            m_t0 = t1;
        }

        // Write out the framerate on the image before displaying it
        string fpsSt("FPS:" + std::to_string((int)m_fpsTracker));
        cv::putText(m_visualization, fpsSt, cv::Point(10, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1, CV_AA);

        // if (!m_faceModelParameters.quiet_mode)
        {
            cv::namedWindow("tracking_result", 1);
            cv::imshow("tracking_result", m_visualization);
            cv::waitKey(1);
        }
    }


} // namespace FaceDetection
} // namespace ar
