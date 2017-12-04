#include "FaceDetectionFilter/FaceDetectionFilter.h"
#include "common/math.h"

namespace ar {
namespace FaceDetectionFilter {


    FaceDetectionFilterParameters::FaceDetectionFilterParameters()
    {
        inputCsvFile = std::string("results.csv");
        outputCsvFile = std::string("filtered.csv");
        filterInterval = 0.05;
        velocityDecay = 0.5;
        velocityDecayDT = 1;
        initialPose = cv::Vec6d(0,0,0,0,0,0);
        initialPoseVelocity = cv::Vec6d(0,0,0,0,0,0);
        initialPoseVariances = cv::Vec6d(1000,1000,1000,10,10,10);
        initialPoseVelocityVariances = cv::Vec6d(10,10,10,3,3,3);
        poseProcessVariances = cv::Vec6d(10,10,10,0.5,0.5,0.5);
        velocityProcessVariances = cv::Vec6d(1,1,1,0.1,0.1,0.1);
        minPoseObservationVariances = cv::Vec6d(50,50,100,0.05,0.05,0.05);
        maxPoseObservationVariances = cv::Vec6d(200,200,400,0.2,0.2,0.2);
    }

    void FaceDetectionFilterParameters::applyArguments(std::vector<std::string>& arguments)
    {
        std::vector<bool> consumeArgument(arguments.size());
        for(int i=0; i < arguments.size(); i++)
        {
            consumeArgument.push_back(false);
        }

        for (size_t i = 1; i < arguments.size(); ++i)
        {
            consumeArgument[i] = false;

            if (arguments[i].compare("-input") == 0)
            {
                consumeArgument[i] = true;
                if (i+1 < arguments.size())
                {
                    inputCsvFile = arguments[i + 1];
                    consumeArgument[i + 1] = true;
                    i++;
                }
            }
            else if (arguments[i].compare("-output") == 0)
            {
                consumeArgument[i] = true;
                if (i+1 < arguments.size())
                {
                    outputCsvFile = arguments[i + 1];
                    consumeArgument[i + 1] = true;
                    i++;
                }
            }
            else if (arguments[i].compare("-filterInterval") == 0)
            {
                consumeArgument[i] = true;
                if (i+1 < arguments.size())
                {
                   std::stringstream sstream(arguments[i + 1]);
                    double value;
                    sstream >> value;
                    filterInterval = value;
                    consumeArgument[i + 1] = true;
                    i++;
                }
            }
            else if (arguments[i].compare("-velocityDecay") == 0)
            {
                consumeArgument[i] = true;
                if (i+2 < arguments.size())
                {
                    std::stringstream sstream1(arguments[i + 1]);
                    std::stringstream sstream2(arguments[i + 2]);
                    sstream1 >> velocityDecay;
                    sstream2 >> velocityDecayDT;
                    consumeArgument[i + 1] = true;
                    consumeArgument[i + 2] = true;
                    i++;
                }
            }
            else if (arguments[i].compare("-initialPose") == 0)
            {
                consumeArgument[i] = true;
                if (i+6 < arguments.size())
                {
                    for(int k=0; k<6; k++)
                    {
                        std::stringstream sstream(arguments[i + 1 + k]);
                        sstream >> initialPose[k];
                        consumeArgument[i + 1 + k] = true;
                    }
                    i++;
                }
            }
            else if (arguments[i].compare("-initialPoseVelocity") == 0)
            {
                consumeArgument[i] = true;
                if (i+6 < arguments.size())
                {
                    for(int k=0; k<6; k++)
                    {
                        std::stringstream sstream(arguments[i + 1 + k]);
                        sstream >> initialPoseVelocity[k];
                        consumeArgument[i + 1 + k] = true;
                    }
                    i++;
                }
            }
            else if (arguments[i].compare("-initialPoseVariances") == 0)
            {
                consumeArgument[i] = true;
                if (i+6 < arguments.size())
                {
                    for(int k=0; k<6; k++)
                    {
                        std::stringstream sstream(arguments[i + 1 + k]);
                        sstream >> initialPoseVariances[k];
                        consumeArgument[i + 1 + k] = true;
                    }
                    i++;
                }
            }
            else if (arguments[i].compare("-initialPoseVelocityVariances") == 0)
            {
                consumeArgument[i] = true;
                if (i+6 < arguments.size())
                {
                    for(int k=0; k<6; k++)
                    {
                        std::stringstream sstream(arguments[i + 1 + k]);
                        sstream >> initialPoseVelocityVariances[k];
                        consumeArgument[i + 1 + k] = true;
                    }
                    i++;
                }
            }
            else if (arguments[i].compare("-poseProcessVariances") == 0)
            {
                consumeArgument[i] = true;
                if (i+6 < arguments.size())
                {
                    for(int k=0; k<6; k++)
                    {
                        std::stringstream sstream(arguments[i + 1 + k]);
                        sstream >> poseProcessVariances[k];
                        consumeArgument[i + 1 + k] = true;
                    }
                    i++;
                }
            }
            else if (arguments[i].compare("-velocityProcessVariances") == 0)
            {
                consumeArgument[i] = true;
                if (i+6 < arguments.size())
                {
                    for(int k=0; k<6; k++)
                    {
                        std::stringstream sstream(arguments[i + 1 + k]);
                        sstream >> velocityProcessVariances[k];
                        consumeArgument[i + 1 + k] = true;
                    }
                    i++;
                }
            }
            else if (arguments[i].compare("-minPoseObservationVariances") == 0)
            {
                consumeArgument[i] = true;
                if (i+6 < arguments.size())
                {
                    for(int k=0; k<6; k++)
                    {
                        std::stringstream sstream(arguments[i + 1 + k]);
                        sstream >> minPoseObservationVariances[k];
                        consumeArgument[i + 1 + k] = true;
                    }
                    i++;
                }
            }
            else if (arguments[i].compare("-maxPoseObservationVariances") == 0)
            {
                consumeArgument[i] = true;
                if (i+6 < arguments.size())
                {
                    for(int k=0; k<6; k++)
                    {
                        std::stringstream sstream(arguments[i + 1 + k]);
                        sstream >> maxPoseObservationVariances[k];
                        consumeArgument[i + 1 + k] = true;
                    }
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

    void FaceDetectionFilterParameters::print()
    {
        std::cout << "inputCsvFile \t" << inputCsvFile << std::endl;
        std::cout << "outputCsvFile \t" << outputCsvFile << std::endl;
        std::cout << "filterInterval \t" << filterInterval << std::endl;
        std::cout << "velocityDecay \t" << velocityDecay << std::endl;
        std::cout << "velocityDecayDT \t" << velocityDecayDT << std::endl;
        std::cout << "initialPose \t" << initialPose << std::endl;
        std::cout << "initialPoseVelocity \t" << initialPoseVelocity << std::endl;
        std::cout << "initialPoseVariances \t" << initialPoseVariances << std::endl;
        std::cout << "initialPoseVelocityVariances \t" << initialPoseVelocityVariances << std::endl;
        std::cout << "poseProcessVariances \t" << poseProcessVariances << std::endl;
        std::cout << "velocityProcessVariances \t" << velocityProcessVariances << std::endl;
        std::cout << "minPoseObservationVariances \t" << minPoseObservationVariances << std::endl;
        std::cout << "maxPoseObservationVariances \t" << maxPoseObservationVariances << std::endl;
    }

    FaceDetectionFilter::FaceDetectionFilter()
        : m_filters{
            FirstOrderScalarFilter(),
            FirstOrderScalarFilter(),
            FirstOrderScalarFilter(),
            FirstOrderScalarFilter(),
            FirstOrderScalarFilter(),
            FirstOrderScalarFilter()
        }
    {}

    FaceDetectionFilter::FaceDetectionFilter(const FaceDetectionFilterParameters& parameters)
        : FaceDetectionFilter(
            parameters.velocityDecay,
            parameters.velocityDecayDT,
            parameters.initialPose,
            parameters.initialPoseVelocity,
            parameters.initialPoseVariances,
            parameters.initialPoseVelocityVariances,
            parameters.poseProcessVariances,
            parameters.velocityProcessVariances,
            parameters.minPoseObservationVariances,
            parameters.maxPoseObservationVariances
        )
    {

    }
    FaceDetectionFilter::FaceDetectionFilter(
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
    )
        : m_minPoseObservationVariances(minPoseObservationVariances)
        , m_maxPoseObservationVariances(maxPoseObservationVariances)
    {
        double velocityDecayPerSecond = FirstOrderScalarFilter::DecayPerSecond(velocityDecay,velocityDecayDT);
        for(int k=0; k<6; k++)
        {
            m_filters[k] = FirstOrderScalarFilter(
                0,                               // initialTime
                initialPose[k],                  // initialValue
                initialPoseVelocity[k],          // initialVelocity
                initialPoseVariances[k],         // initialValueVariance
                initialPoseVelocityVariances[k], // initialVelocityVariance
                poseProcessVariances[0],         // valueProcessVariance
                velocityProcessVariances[0],     // velocityProcessVariance
                m_maxPoseObservationVariances[k],  // observationVariance
                velocityDecayPerSecond           // velocityDecayPerSecond
            );
        }
    }

    cv::Vec6d FaceDetectionFilter::update(double time, cv::Vec6d observedHeadPose, double detectionCertainty)
    {
        if(abs(observedHeadPose[0]) > 1e5 || abs(observedHeadPose[1]) > 1e5 || abs(observedHeadPose[2]) > 1e5)
            return headPose();

        double normalizedCertainty = 1-(detectionCertainty+1)/2;
        if(normalizedCertainty < 1)
        {
            cv::Vec6d observationVariances = m_minPoseObservationVariances + normalizedCertainty * (m_maxPoseObservationVariances - m_minPoseObservationVariances);
            // std::cout << "observedHeadPose \t"<< observedHeadPose << std::endl;
            // std::cout << "observationVariances \t"<< observationVariances << std::endl;
            for(int k=0; k<6; k++)
            {
                m_filters[k].update(time, observedHeadPose[k], observationVariances[k]);
            }
        }
        m_filterTime = time;
        return headPose();
    }

    cv::Vec6d FaceDetectionFilter::predict(double time)
    {
        // std::cout << time - m_filterTime << std::endl;
        for(int k=0; k<6; k++)
        {
            m_filters[k].predict(time);
        }
        m_filterTime = time;
        return headPose();
    }

    cv::Vec6d FaceDetectionFilter::headPose()  const
    {
        return cv::Vec6d(
            m_filters[0].value(),
            m_filters[1].value(),
            m_filters[2].value(),
            m_filters[3].value(),
            m_filters[4].value(),
            m_filters[5].value()
        );
    }  

    double FaceDetectionFilter::getFilterTime()  const
    {
        return m_filterTime;
    }

    const FirstOrderScalarFilter& FaceDetectionFilter::filter(int i) const
    {
        return m_filters[i];
    }


} // namespace FaceDetectionFilter
} // namespace ar
