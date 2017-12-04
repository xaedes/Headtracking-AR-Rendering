#pragma once

#include <opencv2/opencv.hpp>

namespace ar {
namespace FaceDetectionFilter {


    template <typename ScalarType, int NumStates, int NumObservations>
    class KalmanFilter
    {
    public:
        KalmanFilter() = default;
        KalmanFilter(
            const cv::Matx<ScalarType, NumStates, 1>& state,
            const cv::Matx<ScalarType, NumStates, NumStates>& stateTransition,
            const cv::Matx<ScalarType, NumStates, NumStates>& stateCovariances,
            const cv::Matx<ScalarType, NumStates, NumStates>& stateTransitionCovariances,
            const cv::Matx<ScalarType, NumObservations, NumStates>& observationModel,
            const cv::Matx<ScalarType, NumObservations, NumObservations>& observationCovariances
        )
            : m_state(state)
            , m_stateTransition(stateTransition)
            , m_stateCovariances(stateCovariances)
            , m_stateTransitionCovariances(stateTransitionCovariances)
            , m_observationModel(observationModel)
            , m_observationCovariances(observationCovariances)
        {}

        ~KalmanFilter() = default;

        /**
         * @brief      Update Filter with new observation
         *
         * @param[in]  observations          The observations as column vector matrix
         */
        void observe(
            const cv::Matx<ScalarType, NumObservations, 1>& observations 
        )
        {
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            // std::cout << "\t" << "observations" << std::endl << observations << std::endl;
            // std::cout << "\t" << "m_observationModel" << std::endl << m_observationModel << std::endl;
            // std::cout << "\t" << "m_state" << std::endl << m_state << std::endl;
            cv::Matx<ScalarType, NumObservations, 1> innovation = KalmanFilter::innovation(
                observations, 
                m_observationModel,
                m_state
            );
            // std::cout << "\t" << "innovation" << std::endl << innovation << std::endl;
            // std::cout << "\t" << "m_observationCovariances" << std::endl << m_observationCovariances << std::endl;
            // std::cout << "\t" << "m_stateCovariances" << std::endl << m_stateCovariances << std::endl;
            cv::Matx<ScalarType, NumObservations, NumObservations> innovationCovariances = KalmanFilter::innovationCovariances(
                m_observationCovariances,
                m_observationModel,
                m_stateCovariances
            );
            // std::cout << "\t" << "innovationCovariances" << std::endl << innovationCovariances << std::endl;
            cv::Matx<ScalarType, NumStates, NumObservations> kalmanGain = KalmanFilter::kalmanGain(
                m_stateCovariances,
                m_observationModel,
                innovationCovariances
            );
            // std::cout << "\t" << "kalmanGain" << std::endl << kalmanGain << std::endl;
            m_state = KalmanFilter::updatedState(
                m_state,
                kalmanGain,
                innovation
            );
            // std::cout << "\t" << "m_state" << std::endl << m_state << std::endl;
            m_stateCovariances = KalmanFilter::updatedStateCovariances(
                m_stateCovariances,
                kalmanGain,
                innovationCovariances
            );
            // std::cout << "\t" << "m_stateCovariances" << std::endl << m_stateCovariances << std::endl;

            m_observationsPostFitResidual = KalmanFilter::observationsPostFitResidual(
                observations,
                m_observationModel,
                m_state
            );
            // std::cout << "\t" << "m_observationsPostFitResidual" << std::endl << m_observationsPostFitResidual << std::endl;
            
            fixStateCovariancesMatrix();
            // std::cout << "\t" << "m_stateCovariances" << std::endl << m_stateCovariances << std::endl;
            // std::cout << "end of " << __PRETTY_FUNCTION__ << std::endl;

        }

        /**
         * @brief      Update Filter with new observation
         *
         * @param[in]  observations          The observations as column vector matrix
         * @param[in]  observationVariances  The observation covariances
         */
        void observe(
            const cv::Matx<ScalarType, NumObservations, 1>& observations, 
            const cv::Matx<ScalarType, NumObservations, NumObservations>& observationCovariances
        )
        {
            m_observationCovariances = observationCovariances;
            observe(observations);
        }

        /**
         * @brief      Predict the filter state 
         */
        void predict()
        {
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            // std::cout << "\t" << "m_stateTransition" << std::endl << m_stateTransition << std::endl;
            // std::cout << "\t" << "m_state" << std::endl << m_state << std::endl;
            m_state = KalmanFilter::predictState(
                m_stateTransition,
                m_state
            );
            // std::cout << "\t" << "m_state" << std::endl << m_state << std::endl;
            // std::cout << "\t" << "m_stateCovariances" << std::endl << m_stateCovariances << std::endl;
            // std::cout << "\t" << "m_stateTransitionCovariances" << std::endl << m_stateTransitionCovariances << std::endl;
            m_stateCovariances = KalmanFilter::predictStateCovariances(
                m_stateTransition,
                m_stateCovariances,
                m_stateTransitionCovariances
            );
            // std::cout << "\t" << "m_stateCovariances" << std::endl << m_stateCovariances << std::endl;
            fixStateCovariancesMatrix();
            // std::cout << "\t" << "m_stateCovariances" << std::endl << m_stateCovariances << std::endl;
            // std::cout << "end of " << __PRETTY_FUNCTION__ << std::endl;
        }

        /**
         * @brief      Predict the filter state given a state transition matrix
         *
         * @param[in]  stateTransition  The state transition
         */
        void predict(const cv::Matx<ScalarType, NumStates, NumStates>& stateTransition)
        {
            m_stateTransition = stateTransition;
            predict();
        }

        cv::Matx<ScalarType, NumStates, 1> m_state;
        cv::Matx<ScalarType, NumStates, NumStates> m_stateTransition;
        cv::Matx<ScalarType, NumStates, NumStates> m_stateCovariances;
        cv::Matx<ScalarType, NumStates, NumStates> m_stateTransitionCovariances;
        cv::Matx<ScalarType, NumObservations, NumStates> m_observationModel;
        cv::Matx<ScalarType, NumObservations, NumObservations> m_observationCovariances;
        cv::Matx<ScalarType, NumObservations, 1> m_observationsPostFitResidual;

        void print() const
        {
            std::cout << "m_state \t" << m_state << std::endl;
            std::cout << "m_stateTransition \t" << m_stateTransition << std::endl;
            std::cout << "m_stateCovariances \t" << m_stateCovariances << std::endl;
            std::cout << "m_stateTransitionCovariances \t" << m_stateTransitionCovariances << std::endl;
            std::cout << "m_observationModel \t" << m_observationModel << std::endl;
            std::cout << "m_observationCovariances \t" << m_observationCovariances << std::endl;
            std::cout << "m_observationsPostFitResidual \t" << m_observationsPostFitResidual << std::endl;
        }

        void fixStateCovariancesMatrix()
        {
            // beseitigt assymetrie
            m_stateCovariances = (m_stateCovariances + m_stateCovariances.t()) * 0.5;
        }
        
        // actual implementation of Kalman filter math 
        // 
        // according to https://en.wikipedia.org/wiki/Kalman_filter#Details 
        // (as of 30.11.2017)
        
        // prediction:

        static cv::Matx<ScalarType, NumStates, 1> predictState(
            const cv::Matx<ScalarType, NumStates, NumStates>& stateTransition,
            const cv::Matx<ScalarType, NumStates, 1>& previousState
        )
        {
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            return stateTransition * previousState;
        }

        static cv::Matx<ScalarType, NumStates, NumStates> predictStateCovariances(
            const cv::Matx<ScalarType, NumStates, NumStates>& stateTransition,
            const cv::Matx<ScalarType, NumStates, NumStates>& previousStateCovariances,
            const cv::Matx<ScalarType, NumStates, NumStates>& stateTransitionCovariances
        )
        {
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            return stateTransition * previousStateCovariances * stateTransition.t() + stateTransitionCovariances;
        }

        // update:
        
        static cv::Matx<ScalarType, NumObservations, 1> innovation(
            const cv::Matx<ScalarType, NumObservations, 1>& observations,
            const cv::Matx<ScalarType, NumObservations, NumStates>& observationModel,
            const cv::Matx<ScalarType, NumStates, 1>& state
        )
        {
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            return observations - observationModel * state;
        }

        static cv::Matx<ScalarType, NumObservations, NumObservations> innovationCovariances(
            const cv::Matx<ScalarType, NumObservations, NumObservations>& observationCovariances,
            const cv::Matx<ScalarType, NumObservations, NumStates>& observationModel,
            const cv::Matx<ScalarType, NumStates, NumStates>& stateCovariances
        )
        {
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            return observationCovariances + observationModel * stateCovariances * observationModel.t();
        }

        static cv::Matx<ScalarType, NumStates, NumObservations> kalmanGain(
            const cv::Matx<ScalarType, NumStates, NumStates>& stateCovariances,
            const cv::Matx<ScalarType, NumObservations, NumStates>& observationModel,
            const cv::Matx<ScalarType, NumObservations, NumObservations>& innovationCovariances
        )
        {
            cv::Matx<ScalarType, NumObservations, NumObservations> invInnovationCovariances = innovationCovariances.inv();
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            // std::cout << "\t" << "innovationCovariances" << std::endl << innovationCovariances << std::endl;
            // std::cout << "\t" << "invInnovationCovariances" << std::endl << invInnovationCovariances << std::endl;
            // std::cout << "\t" << "invInnovationCovariances.dot(innovationCovariances)" << std::endl << invInnovationCovariances.dot(innovationCovariances) << std::endl;
            // std::cout << "end of " << __PRETTY_FUNCTION__ << std::endl;
            return stateCovariances * observationModel.t() * invInnovationCovariances;
        }

        static cv::Matx<ScalarType, NumStates, 1> updatedState(
            const cv::Matx<ScalarType, NumStates, 1>& previousState,
            const cv::Matx<ScalarType, NumStates, NumObservations>& kalmanGain,
            const cv::Matx<ScalarType, NumObservations, 1>& innovation
        )
        {
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            return previousState + kalmanGain * innovation;
        }

        static cv::Matx<ScalarType, NumStates, NumStates> updatedStateCovariances(
            const cv::Matx<ScalarType, NumStates, NumStates>& previousStateCovariances,
            const cv::Matx<ScalarType, NumStates, NumObservations>& kalmanGain,
            const cv::Matx<ScalarType, NumObservations, NumObservations>& innovationCovariances
        )
        {
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            return previousStateCovariances - kalmanGain * innovationCovariances * kalmanGain.t();
        }

        static cv::Matx<ScalarType, NumObservations, 1> observationsPostFitResidual(
            const cv::Matx<ScalarType, NumObservations, 1>& observations,
            const cv::Matx<ScalarType, NumObservations, NumStates>& observationModel,
            const cv::Matx<ScalarType, NumStates, 1>& state
        )
        {
            // std::cout << __PRETTY_FUNCTION__ << std::endl;
            return observations - observationModel * state;
        }



    };

} // namespace FaceDetectionFilter
} // namespace ar

