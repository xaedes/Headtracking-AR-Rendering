#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch/catch.hpp"
#include "catch/EqualsMat.hpp"
#include <type_traits>
#include <memory>

#include "FaceDetectionFilter/FaceDetectionFilter.h"

using namespace ar::FaceDetectionFilter;

TEST_CASE("FaceDetectionFilter", "[FaceDetectionFilter]" ) 
{
    FaceDetectionFilterParameters filterParameters;
    FaceDetectionFilter filter(filterParameters);
    filter.update(1,cv::Vec6d(1,1,1,1,1,1),0.5);
    filter.predict(2);

    SECTION("FaceDetectionFilter& operator=(FaceDetectionFilter&,const FaceDetectionFilter&)")
    {
        FaceDetectionFilter filter2 = filter;
    
        for(int i=0; i<6; i++)
        {
            REQUIRE(filter.getFilterTime() == filter2.getFilterTime());
            REQUIRE(filter.headPose()[i] == filter2.headPose()[i]);
            REQUIRE(filter.filter(i).getFilterTime() == filter2.filter(i).getFilterTime());
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_state),                       EqualsMat(cv::Mat(filter2.filter(i).kalman().m_state)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_stateTransition),             EqualsMat(cv::Mat(filter2.filter(i).kalman().m_stateTransition)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_stateCovariances),            EqualsMat(cv::Mat(filter2.filter(i).kalman().m_stateCovariances)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_stateTransitionCovariances),  EqualsMat(cv::Mat(filter2.filter(i).kalman().m_stateTransitionCovariances)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_observationModel),            EqualsMat(cv::Mat(filter2.filter(i).kalman().m_observationModel)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_observationCovariances),      EqualsMat(cv::Mat(filter2.filter(i).kalman().m_observationCovariances)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_observationsPostFitResidual), EqualsMat(cv::Mat(filter2.filter(i).kalman().m_observationsPostFitResidual)) );
        }
    }

    SECTION("FaceDetectionFilter& operator=(FaceDetectionFilter&,const FaceDetectionFilter&)")
    {
        FaceDetectionFilter filter2;
        filter2 = filter;
    
        for(int i=0; i<6; i++)
        {
            REQUIRE(filter.getFilterTime() == filter2.getFilterTime());
            REQUIRE(filter.headPose()[i] == filter2.headPose()[i]);
            REQUIRE(filter.filter(i).getFilterTime() == filter2.filter(i).getFilterTime());
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_state),                       EqualsMat(cv::Mat(filter2.filter(i).kalman().m_state)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_stateTransition),             EqualsMat(cv::Mat(filter2.filter(i).kalman().m_stateTransition)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_stateCovariances),            EqualsMat(cv::Mat(filter2.filter(i).kalman().m_stateCovariances)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_stateTransitionCovariances),  EqualsMat(cv::Mat(filter2.filter(i).kalman().m_stateTransitionCovariances)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_observationModel),            EqualsMat(cv::Mat(filter2.filter(i).kalman().m_observationModel)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_observationCovariances),      EqualsMat(cv::Mat(filter2.filter(i).kalman().m_observationCovariances)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_observationsPostFitResidual), EqualsMat(cv::Mat(filter2.filter(i).kalman().m_observationsPostFitResidual)) );
        }
    }


    SECTION("FaceDetectionFilter& operator=(FaceDetectionFilter&,const FaceDetectionFilter&)")
    {
        std::unique_ptr<FaceDetectionFilter> filter2(new FaceDetectionFilter());
        *filter2 = filter;
    
        for(int i=0; i<6; i++)
        {
            REQUIRE(filter.getFilterTime() == filter2->getFilterTime());
            REQUIRE(filter.headPose()[i] == filter2->headPose()[i]);
            REQUIRE(filter.filter(i).getFilterTime() == filter2->filter(i).getFilterTime());
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_state),                       EqualsMat(cv::Mat(filter2->filter(i).kalman().m_state)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_stateTransition),             EqualsMat(cv::Mat(filter2->filter(i).kalman().m_stateTransition)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_stateCovariances),            EqualsMat(cv::Mat(filter2->filter(i).kalman().m_stateCovariances)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_stateTransitionCovariances),  EqualsMat(cv::Mat(filter2->filter(i).kalman().m_stateTransitionCovariances)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_observationModel),            EqualsMat(cv::Mat(filter2->filter(i).kalman().m_observationModel)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_observationCovariances),      EqualsMat(cv::Mat(filter2->filter(i).kalman().m_observationCovariances)) );
            REQUIRE_THAT( cv::Mat(filter.filter(i).kalman().m_observationsPostFitResidual), EqualsMat(cv::Mat(filter2->filter(i).kalman().m_observationsPostFitResidual)) );
        }
    }


}
