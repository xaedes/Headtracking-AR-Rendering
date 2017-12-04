#pragma once

#include "common/Maybe.h"

namespace ar {
namespace FaceDetectionFilter {

    /**
     * @brief      Class for second order kinematic scalar filter.
     *
     *             Filters a scalar value by estimating its velocity and current
     *             value, with only observing the value.
     */
    class FirstOrderScalarGainFilter
    {
    public:
        static double GainPerSecond(double gainPerDt, float dt);
        static double DecayPerSecond(double decayPerDt, float dt);

        FirstOrderScalarGainFilter();
        FirstOrderScalarGainFilter(
            double initialTime, 
            double initialValue, 
            double velocityDecayPerSecond, 
            double velocityObservationGainPerSecond,
            double valueObservationGainPerSecond
        );
        ~FirstOrderScalarGainFilter() = default;

        /**
         * @brief      Update Filter with new observation of value
         *
         * @param[in]  time                 The time of the observation
         * @param[in]  value                The observed value
         * @param[in]  observationCertainty  The certainty of the observation ranging from [0..1]
         *
         * @return     The updated value
         */
        double update(double time, double observedValue, double observationCertainty);

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

    protected:


        double m_filterTime;
        double m_value;
        double m_velocity;

        double m_velocityDecayPerSecond;
        double m_velocityObservationGainPerSecond;
        double m_valueObservationGainPerSecond;

        common::Maybe<double> m_valueStateAtLastObservation;
        common::Maybe<double> m_timeLastObservation;


    };

} // namespace FaceDetectionFilter
} // namespace ar
