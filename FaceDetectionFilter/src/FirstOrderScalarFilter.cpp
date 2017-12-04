#include "FaceDetectionFilter/FirstOrderScalarFilter.h"
#include "common/math.h"

namespace ar {
namespace FaceDetectionFilter {

    double FirstOrderScalarFilter::DecayPerSecond(double decayPerDt, float dt)
    {
        if(decayPerDt == 1) return 1;
        return exp(log(decayPerDt) / dt);
    }
    FirstOrderScalarFilter::FirstOrderScalarFilter()
        : FirstOrderScalarFilter(
            0, // initialTime
            0, // initialValue
            0, // initialVelocity
            10, // initialValueVariance
            10, // initialVelocityVariance
            1, // valueProcessVariance
            2, // velocityProcessVariance
            1, // observationVariance
            0 // velocityDecayPerSecond
        )
    {}
    FirstOrderScalarFilter::FirstOrderScalarFilter(
        double initialTime, 
        double initialValue, 
        double initialVelocity, 
        double initialValueVariance,
        double initialVelocityVariance,
        double valueProcessVariance, 
        double velocityProcessVariance,
        double observationVariance,
        double velocityDecayPerSecond
    )
        : m_velocityDecayPerSecond(velocityDecayPerSecond)
        , m_filterTime(initialTime)
    {
        m_observation(0) = 0;
        m_kalman.m_state(0) = initialValue;
        m_kalman.m_state(1) = initialVelocity;
        m_kalman.m_stateTransition = stateTransition(1);
        m_kalman.m_stateCovariances = cv::Matx<double, 2, 2>(
            initialValueVariance, 0,
            0, initialVelocityVariance
        );
        m_kalman.m_stateTransitionCovariances = stateTransitionCovariances(valueProcessVariance,velocityProcessVariance);
        m_kalman.m_observationModel = cv::Matx<double, 1, 2>(
            1,0
        );
        m_kalman.m_observationCovariances = observationCovariances(observationVariance);
    }

    cv::Matx<double, 2, 2> FirstOrderScalarFilter::stateTransition(double dt) const
    {
        return cv::Matx<double, 2, 2>(
            1, dt,
            0, 1/*pow(m_velocityDecayPerSecond,dt)*/
        );
    }

    cv::Matx<double, 2, 2> FirstOrderScalarFilter::stateTransitionCovariances(double valueProcessVariance, double velocityProcessVariance) const
    {
        return cv::Matx<double, 2, 2>(
            valueProcessVariance, 0,
            0, velocityProcessVariance
        );
    }

    cv::Matx<double, 1, 1> FirstOrderScalarFilter::observationCovariances(double observationValueVariance) const
    {
        return cv::Matx<double, 1, 1>(
            observationValueVariance
        );
    }

    double FirstOrderScalarFilter::update(double time, double observation, double observationValueVariance)
    {
        double dt = time - m_filterTime;
        if (dt >= 0)
        {
            predict(time);
            m_observation(0) = observation;
            m_kalman.observe(m_observation, observationCovariances(observationValueVariance));

            m_filterTime = time;
        }
        return value();
    }
    double FirstOrderScalarFilter::predict(double time)
    {
        double dt = time - m_filterTime;
        if (dt >= 0)
        {
            m_kalman.predict(stateTransition(dt));

            m_filterTime = time;
        }
        return value();
    }

    double FirstOrderScalarFilter::getFilterTime() const
    {
        return m_filterTime;
    }

    double FirstOrderScalarFilter::value() const
    {
        return m_kalman.m_state(0);
    }

    double FirstOrderScalarFilter::velocity() const
    {
        return m_kalman.m_state(1);
    }

    void FirstOrderScalarFilter::print() const
    {
        m_kalman.print();
    }

    const KalmanFilter<double, 2, 1>& FirstOrderScalarFilter::kalman() const
    {
        return m_kalman;
    }

} // namespace FaceDetectionFilter
} // namespace ar
