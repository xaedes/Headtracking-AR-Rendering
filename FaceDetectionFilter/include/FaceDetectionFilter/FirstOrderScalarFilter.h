#pragma once

#include "FaceDetectionFilter/KalmanFilter.h"

namespace ar {
namespace FaceDetectionFilter {

    /**
     * @brief      Class for second order kinematic scalar filter.
     *
     *             Filters a scalar value by estimating its velocity and current
     *             value, with only observing the value.
     */
    class FirstOrderScalarFilter
    {
    public:
        static double DecayPerSecond(double decayPerDt, float dt);
        FirstOrderScalarFilter();
        FirstOrderScalarFilter(
            double initialTime, 
            double initialValue, 
            double initialVelocity, 
            double initialValueVariance,
            double initialVelocityVariance,
            double valueProcessVariance, 
            double velocityProcessVariance,
            double observationVariance,
            double velocityDecayPerSecond
        );
        ~FirstOrderScalarFilter() = default;

        /**
         * @brief      Update Filter with new observation of value
         *
         * @param[in]  time                 The time of the observation
         * @param[in]  value                The observed value
         * @param[in]  observationVariance  The variance of the observation
         *
         * @return     The updated value
         */
        double update(double time, double observation, double observationVariance);

        /**
         * @brief      Predict the filter state for the given time
         *
         * @param[in]  time  The time
         *
         * @return     the predicted value 
         */
        double predict(double time);

        /**
         * @brief      Gets the current filter time.
         *
         * @return     The filter time.
         */
        double getFilterTime() const;

        /**
         * @brief      Returns the currently estimated value
         *             
         * @return     currently estimated value of the filter
         */
        double value() const;

        /**
         * @brief      Returns the currently estimated velocity
         *
         * @return     currently estimated velocity of the filter
         */
        double velocity() const;

        void print() const;

        const KalmanFilter<double, 2, 1>& kalman() const;
    protected:

        cv::Matx<double, 2, 2> stateTransition(double dt) const;
        cv::Matx<double, 1, 1> observationCovariances(double observationValueVariance) const;
        cv::Matx<double, 2, 2> stateTransitionCovariances(double valueProcessVariance, double velocityProcessVariance) const;

        cv::Matx<double, 1, 1> m_observation;

        double m_velocityDecayPerSecond;
        double m_filterTime;
        KalmanFilter<double, 2, 1> m_kalman;
    };

} // namespace FaceDetectionFilter
} // namespace ar
