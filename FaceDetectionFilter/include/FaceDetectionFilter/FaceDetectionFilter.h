#pragma once

#include <opencv2/opencv.hpp>
#include "FaceDetectionFilter/FirstOrderScalarFilter.h"
// #include "FaceDetectionFilter/FirstOrderScalarGainFilter.h"

namespace ar {
namespace FaceDetectionFilter {


    struct FaceDetectionFilterParameters
    {
        std::string inputCsvFile;
        std::string outputCsvFile;
        double filterInterval;

        double velocityDecay;
        double velocityDecayDT;
        cv::Vec6d initialPose;
        cv::Vec6d initialPoseVelocity;
        cv::Vec6d initialPoseVariances;
        cv::Vec6d initialPoseVelocityVariances;
        cv::Vec6d poseProcessVariances;
        cv::Vec6d velocityProcessVariances;
        cv::Vec6d minPoseObservationVariances;
        cv::Vec6d maxPoseObservationVariances;

        FaceDetectionFilterParameters();
        void applyArguments(std::vector<std::string>& arguments);
        void print();

    };
    class FaceDetectionFilter
    {
    public:
        FaceDetectionFilter();
        // FaceDetectionFilter(const FaceDetectionFilter& copyFrom);
        FaceDetectionFilter(const FaceDetectionFilterParameters& parameters);
        FaceDetectionFilter(
            double velocityDecay, 
            double velocityDecayDT,
            cv::Vec6d initialPose,
            cv::Vec6d initialPoseVelocity,
            cv::Vec6d initialPoseVariances,
            cv::Vec6d initialPoseVelocityVariances,
            cv::Vec6d poseProcessVariances,
            cv::Vec6d velocityProcessVariances,
            cv::Vec6d minPoseObservationVariances,
            cv::Vec6d maxPoseObservationVariances
        );
        ~FaceDetectionFilter() = default;

        /**
         * @brief      Update Filter with new observations
         *
         * @param[in]  time                The time of the observation
         * @param[in]  observedHeadPose    The observed head pose
         * @param[in]  detectionCertainty  The detection certainty ranging from
         *                                 -1 (best) to +1 (worst)
         * @param[in]  headPose  The observed head pose
         *
         * @return     The current estimate for headPose
         */
        cv::Vec6d update(double time, cv::Vec6d observedHeadPose, double detectionCertainty);

        /**
         * @brief      Predict the filter state for the given time
         *
         * @param[in]  time  The time
         *
         * @return     The current estimate for headPose
         */
        cv::Vec6d predict(double time);

        /**
         * @brief      The current filter state
         *
         * @return     Head Pose as follows [x,y,z,eulerX,eulerY,eulerZ]
         */
        cv::Vec6d headPose() const;


        /**
         * @brief      Gets the current filter time.
         *
         * @return     The filter time.
         */
        double getFilterTime() const;

        const FirstOrderScalarFilter& filter(int i) const;
        
    protected:
        FirstOrderScalarFilter m_filters[6];
        double m_filterTime;
        cv::Vec6d m_minPoseObservationVariances;
        cv::Vec6d m_maxPoseObservationVariances;
    };

} // namespace FaceDetectionFilter
} // namespace ar
