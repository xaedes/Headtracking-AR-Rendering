#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch/catch.hpp"
#include "catch/EqualsMat.hpp"
#include <type_traits>

#include "FaceDetectionFilter/FirstOrderScalarFilter.h"

using namespace ar::FaceDetectionFilter;

TEST_CASE("FaceDetectionFilter::FirstOrderScalarFilter", "[FaceDetectionFilter][FirstOrderScalarFilter]" ) 
{
    FirstOrderScalarFilter filter;
    filter.update(1,1,1);
    filter.predict(2);

    SECTION("FirstOrderScalarFilter& operator=(FirstOrderScalarFilter&,const FirstOrderScalarFilter&)")
    {
        FirstOrderScalarFilter filter2 = filter;
        
        REQUIRE(filter.getFilterTime() == filter2.getFilterTime());
        REQUIRE_THAT( cv::Mat(filter.kalman().m_state),                       EqualsMat(cv::Mat(filter2.kalman().m_state)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_stateTransition),             EqualsMat(cv::Mat(filter2.kalman().m_stateTransition)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_stateCovariances),            EqualsMat(cv::Mat(filter2.kalman().m_stateCovariances)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_stateTransitionCovariances),  EqualsMat(cv::Mat(filter2.kalman().m_stateTransitionCovariances)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_observationModel),            EqualsMat(cv::Mat(filter2.kalman().m_observationModel)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_observationCovariances),      EqualsMat(cv::Mat(filter2.kalman().m_observationCovariances)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_observationsPostFitResidual), EqualsMat(cv::Mat(filter2.kalman().m_observationsPostFitResidual)) );
    }

    SECTION("FirstOrderScalarFilter& operator=(FirstOrderScalarFilter&,const FirstOrderScalarFilter&)")
    {
        FirstOrderScalarFilter filter2;
        filter2 = filter;
        
        REQUIRE(filter.getFilterTime() == filter2.getFilterTime());
        REQUIRE_THAT( cv::Mat(filter.kalman().m_state),                       EqualsMat(cv::Mat(filter2.kalman().m_state)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_stateTransition),             EqualsMat(cv::Mat(filter2.kalman().m_stateTransition)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_stateCovariances),            EqualsMat(cv::Mat(filter2.kalman().m_stateCovariances)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_stateTransitionCovariances),  EqualsMat(cv::Mat(filter2.kalman().m_stateTransitionCovariances)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_observationModel),            EqualsMat(cv::Mat(filter2.kalman().m_observationModel)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_observationCovariances),      EqualsMat(cv::Mat(filter2.kalman().m_observationCovariances)) );
        REQUIRE_THAT( cv::Mat(filter.kalman().m_observationsPostFitResidual), EqualsMat(cv::Mat(filter2.kalman().m_observationsPostFitResidual)) );
    }


}
