#pragma once

#include <opencv2/opencv.hpp>

#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>

#include "FaceDetection/FaceDetection.h"
#include "FaceDetectionFilter/FaceDetectionFilter.h"


namespace ar {
namespace FaceDetectionComponent {


/// \brief      Class for the main program that.
class FaceDetectionComponent
{
public:
    FaceDetectionComponent();
    ~FaceDetectionComponent();

    void start(int argc, char* argv[]);
    void stop();
    
    void runFaceDetection();

    void pushImage(const cv::Mat_<cv::Vec3b>& image);

    cv::Mat visualizedImage();
    cv::Mat visualizedImage(const cv::Mat_<cv::Vec3b>& colorFrame);
    cv::Vec6d headPose();

    double timeSinceStart() const;

protected:
    int m_argc;
    char** m_argv;
    void wait(double seconds);
    void startTime();

    std::chrono::time_point<std::chrono::system_clock> m_startTime;

    bool m_running;
    std::thread m_faceDetectionThread;

    std::unique_ptr<FaceDetection::FaceDetection> m_faceDetection;

    // receives new observations, now intermediate predictions are made, to keep the timeline in order
    FaceDetectionFilter::FaceDetectionFilter m_faceDetectionObservationFilter;
    // is used to produce intermediate steps. is equal to observation filter right after observation
    FaceDetectionFilter::FaceDetectionFilter m_faceDetectionPredictionFilter;
    std::mutex m_faceDetectionPredictionFilterMutex;

    void resetObservationFilterIfNan(const FaceDetectionFilter::FaceDetectionFilterParameters& faceDetectionFilterParameters);
    void updatePredictionFilter();

    std::queue<std::pair<double,cv::Mat_<cv::Vec3b>>> m_imageQueue;
    std::mutex m_imageQueueMutex;

    cv::Mat_<cv::Vec3b> m_lastImage;
    std::mutex m_lastImageMutex;

    cv::Mat visualizeTracking(
        const cv::Mat_<cv::Vec3b>& colorFrame, 
        cv::Vec6d headPose
    );
};

} // namespace FaceDetectionComponent
} // namespace ar
