#include "FaceDetectionFilter/FirstOrderScalarGainFilter.h"
#include "common/math.h"
#include <iostream>

namespace ar {
namespace FaceDetectionFilter {

    double FirstOrderScalarGainFilter::GainPerSecond(double gainPerDt, float dt)
    {
        if(gainPerDt == 0) return 0;
        if(gainPerDt == 1) return 1;
        return exp(log(gainPerDt) / dt);
    }

    double FirstOrderScalarGainFilter::DecayPerSecond(double decayPerDt, float dt)
    {
        if(decayPerDt == 1) return 1;
        return exp(log(decayPerDt) / dt);
    }


    FirstOrderScalarGainFilter::FirstOrderScalarGainFilter()
        : FirstOrderScalarGainFilter(
            0,
            0,
            DecayPerSecond(0.5, 1.),
            GainPerSecond(0.999, 1./30.),
            GainPerSecond(0.9999, 1./30.)
        )
    {}
    FirstOrderScalarGainFilter::FirstOrderScalarGainFilter(
        double initialTime, 
        double initialValue, 
        double velocityDecayPerSecond, 
        double velocityObservationGainPerSecond,
        double valueObservationGainPerSecond
    )
        : m_filterTime(initialTime)
        , m_value(initialValue)
        , m_velocity(0)
        , m_velocityDecayPerSecond(velocityDecayPerSecond)
        , m_velocityObservationGainPerSecond(velocityObservationGainPerSecond)
        , m_valueObservationGainPerSecond(valueObservationGainPerSecond)
    {

    }


    double FirstOrderScalarGainFilter::update(double time, double observedValue, double observationCertainty)
    {
        if (m_filterTime == 0) 
        {
            m_filterTime = time;
            m_value = observedValue;
        }
        double dt = time - m_filterTime;
        if (dt > 0)
        {
            predict(time);

            double valueObservationGain = (1-observationCertainty) * pow(m_valueObservationGainPerSecond, dt);

            double lastValue = m_value;
            if(valueObservationGain < 0 || valueObservationGain > 1)
            {
                std::cout << "---" << std::endl;
                std::cout << "dt\t" << dt << std::endl;
                std::cout << "m_valueObservationGainPerSecond\t" << m_valueObservationGainPerSecond << std::endl;
                std::cout << "observationCertainty\t" << observationCertainty << std::endl;
                std::cout << "pow(m_valueObservationGainPerSecond, dt)\t" << pow(m_valueObservationGainPerSecond, dt) << std::endl;
                std::cout << "valueObservationGain\t" << valueObservationGain << std::endl;
            }
            m_value = observedValue  * (1-valueObservationGain) + valueObservationGain * m_value;

            if(m_valueStateAtLastObservation && m_timeLastObservation)
            {
                double timeSinceLastObservation = time - m_timeLastObservation.value();
                double observedVelocity = (m_value - m_valueStateAtLastObservation.value()) / timeSinceLastObservation;
                double velocityObservationGain = (1-observationCertainty) * pow(m_velocityObservationGainPerSecond, timeSinceLastObservation);
                m_velocity = observedVelocity  * (1-velocityObservationGain) + velocityObservationGain * m_velocity;
                if(velocityObservationGain < 0 || velocityObservationGain > 1)
                {
                    std::cout << "---" << std::endl;
                    std::cout << "timeSinceLastObservation\t" << timeSinceLastObservation << std::endl;
                    std::cout << "m_velocityObservationGainPerSecond\t" << m_velocityObservationGainPerSecond << std::endl;
                    std::cout << "observationCertainty\t" << observationCertainty << std::endl;
                    std::cout << "pow(m_velocityObservationGainPerSecond, timeSinceLastObservation)\t" << pow(m_velocityObservationGainPerSecond, timeSinceLastObservation) << std::endl;
                    std::cout << "velocityObservationGain\t" << velocityObservationGain << std::endl;
                }
            }
            m_valueStateAtLastObservation.just(m_value);
            m_timeLastObservation.just(time);

            m_filterTime = time;
        }
        return value();
    }

    double FirstOrderScalarGainFilter::predict(double time)
    {
        if (m_filterTime == 0) m_filterTime = time;
        double dt = time - m_filterTime;
        if (dt > 0)
        {
            
            m_value = m_value + dt * m_velocity;

            double velocityDecay = pow(m_velocityDecayPerSecond, dt);
            m_velocity *= (1 - velocityDecay);

            if(velocityDecay < 0 || velocityDecay > 1)
            {

                std::cout << "---" << std::endl;
                std::cout << "m_velocityDecayPerSecond\t" << m_velocityDecayPerSecond << std::endl;
                std::cout << "dt\t" << dt << std::endl;
                std::cout << "pow(m_velocityDecayPerSecond, dt)\t" << pow(m_velocityDecayPerSecond, dt) << std::endl;
                std::cout << "velocityDecay\t" << velocityDecay << std::endl;
            }

            m_filterTime = time;
        }
        return value();
    }

    double FirstOrderScalarGainFilter::getFilterTime() const
    {
        return m_filterTime;
    }

    double FirstOrderScalarGainFilter::value() const
    {
        return m_value;
    }

    double FirstOrderScalarGainFilter::velocity() const
    {
        return m_velocity;
    }


} // namespace FaceDetectionFilter
} // namespace ar
