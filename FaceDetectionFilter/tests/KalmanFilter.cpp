#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch/catch.hpp"
#include "catch/EqualsMat.hpp"
#include <type_traits>

#include "FaceDetectionFilter/KalmanFilter.h"

using namespace ar::FaceDetectionFilter;

TEST_CASE("FaceDetectionFilter::KalmanFilter", "[FaceDetectionFilter][KalmanFilter]" ) 
{
    KalmanFilter<double, 3, 2> kalman;
    int k;
    
    k=1;
    for(int i=0; i<kalman.m_state.rows; i++)
    {
        kalman.m_state(i) = k++;
    }
    k=1;
    for(int i=0; i<kalman.m_stateTransition.rows; i++)
    {
        for(int j=0; j<kalman.m_stateTransition.cols; j++)
        {
            kalman.m_stateTransition(i,j) = k++;
        }
    }
    k=1;
    for(int i=0; i<kalman.m_stateCovariances.rows; i++)
    {
        for(int j=0; j<kalman.m_stateCovariances.cols; j++)
        {
            kalman.m_stateCovariances(i,j) = k++;
            kalman.m_stateTransitionCovariances(i,j) = k++;
        }
    }
    k=1;
    for(int i=0; i<kalman.m_observationModel.rows; i++)
    {
        for(int j=0; j<kalman.m_observationModel.cols; j++)
        {
            kalman.m_observationModel(i,j) = k++;
        }
    }
    for(int i=0; i<kalman.m_observationCovariances.rows; i++)
    {
        for(int j=0; j<kalman.m_observationCovariances.cols; j++)
        {
            kalman.m_observationCovariances(i,j) = k++;
        }
    }
    for(int i=0; i<kalman.m_observationsPostFitResidual.rows; i++)
    {
        kalman.m_observationsPostFitResidual(i) = k++;
    }

    SECTION("KalmanFilter& operator=(KalmanFilter&,const KalmanFilter&)")
    {
        KalmanFilter<double, 3, 2> kalman2 = kalman;
        
        REQUIRE_THAT( cv::Mat(kalman.m_state),                       EqualsMat(cv::Mat(kalman2.m_state)) );
        REQUIRE_THAT( cv::Mat(kalman.m_stateTransition),             EqualsMat(cv::Mat(kalman2.m_stateTransition)) );
        REQUIRE_THAT( cv::Mat(kalman.m_stateCovariances),            EqualsMat(cv::Mat(kalman2.m_stateCovariances)) );
        REQUIRE_THAT( cv::Mat(kalman.m_stateTransitionCovariances),  EqualsMat(cv::Mat(kalman2.m_stateTransitionCovariances)) );
        REQUIRE_THAT( cv::Mat(kalman.m_observationModel),            EqualsMat(cv::Mat(kalman2.m_observationModel)) );
        REQUIRE_THAT( cv::Mat(kalman.m_observationCovariances),      EqualsMat(cv::Mat(kalman2.m_observationCovariances)) );
        REQUIRE_THAT( cv::Mat(kalman.m_observationsPostFitResidual), EqualsMat(cv::Mat(kalman2.m_observationsPostFitResidual)) );
    }

    SECTION("KalmanFilter& operator=(KalmanFilter&,const KalmanFilter&)")
    {
        KalmanFilter<double, 3, 2> kalman2;
        kalman2 = kalman;
        
        REQUIRE_THAT( cv::Mat(kalman.m_state),                       EqualsMat(cv::Mat(kalman2.m_state)) );
        REQUIRE_THAT( cv::Mat(kalman.m_stateTransition),             EqualsMat(cv::Mat(kalman2.m_stateTransition)) );
        REQUIRE_THAT( cv::Mat(kalman.m_stateCovariances),            EqualsMat(cv::Mat(kalman2.m_stateCovariances)) );
        REQUIRE_THAT( cv::Mat(kalman.m_stateTransitionCovariances),  EqualsMat(cv::Mat(kalman2.m_stateTransitionCovariances)) );
        REQUIRE_THAT( cv::Mat(kalman.m_observationModel),            EqualsMat(cv::Mat(kalman2.m_observationModel)) );
        REQUIRE_THAT( cv::Mat(kalman.m_observationCovariances),      EqualsMat(cv::Mat(kalman2.m_observationCovariances)) );
        REQUIRE_THAT( cv::Mat(kalman.m_observationsPostFitResidual), EqualsMat(cv::Mat(kalman2.m_observationsPostFitResidual)) );
    }


}
