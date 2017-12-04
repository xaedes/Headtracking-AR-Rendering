#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include <OpenFace/LandmarkDetector/LandmarkCoreIncludes.h>
#include <OpenFace/FaceAnalyser/Face_utils.h>
#include <OpenFace/FaceAnalyser/FaceAnalyser.h>
#include <OpenFace/GazeAnalyser/GazeEstimation.h>

namespace ar {
namespace FaceDetection {

    struct FaceDetectionAdditionalParameters
    {
        FaceDetectionAdditionalParameters();
        float scale;
        bool visualization;
    };

    class FaceDetection
    {
    public:
        FaceDetection();
        FaceDetection(int argc, char **argv);
        ~FaceDetection();

        #warning rename FaceDetection::feedColorFrame to FaceDetection::processImage
        void feedColorFrame(const cv::Mat_<cv::Vec3b>& colorFrame);
        cv::Mat& visualization();
        cv::Vec6d headPose();
        double detectionCertainty();

    protected:
        
        int m_frameCount;
        
        LandmarkDetector::FaceModelParameters m_faceModelParameters;
        FaceDetectionAdditionalParameters m_additionalParameters;
        // std::unique_ptr<FaceAnalysis::FaceAnalyserParameters> m_faceAnalysisParameters;

        std::unique_ptr<LandmarkDetector::CLNF> m_faceModel;
        // std::unique_ptr<FaceAnalysis::FaceAnalyser> m_faceAnalyser;
        void init();
        void applyArguments(
            LandmarkDetector::FaceModelParameters& parameters, 
            FaceDetectionAdditionalParameters& addParameters, 
            std::vector<std::string>& arguments
        ) const;

        cv::Vec6d m_headPose;

        void visualizeTracking(
            const cv::Mat_<cv::Vec3b>& colorFrame, 
            cv::Point3f gazeDirection0, 
            cv::Point3f gazeDirection1, 
            double fx, 
            double fy, 
            double cx, 
            double cy
        );
        cv::Mat m_visualization;
        //  for tracking timing information for visualisation
        double m_fpsTracker;
        int64 m_t0;

    };
    
    #warning remove FaceDetection& operator<< and operator>>
    FaceDetection& operator<<(FaceDetection& faceDetection, const cv::Mat_<cv::Vec3b>& colorFrame);
    FaceDetection& operator>>(FaceDetection& faceDetection, cv::Mat& visualizedFrame);
    

} // namespace FaceDetection
} // namespace ar
