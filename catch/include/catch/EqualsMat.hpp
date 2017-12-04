
#include <opencv2/opencv.hpp>
#include "catch/catch.hpp"

// The matcher class
class MatMatcher : public Catch::MatcherBase<cv::Mat> 
{
public:
    MatMatcher(cv::Mat mat, float_t eps=1e-3) 
        : m_mat(mat)
        , m_epsilon(eps)
    {}

    MatMatcher epsilon(float_t eps)
    {
        m_epsilon = eps;
    }

    // Performs the test for this matcher
    virtual bool match( cv::Mat const& otherMat ) const override {
        return cv::countNonZero(cv::abs(otherMat - m_mat) < m_epsilon) == m_mat.total();
    }

    // Produces a string describing what this matcher does. It should
    // include any provided data (the begin/ end in this case) and
    // be written as if it were stating a fact (in the output it will be
    // preceded by the value under test).
    virtual std::string describe() const {
        std::ostringstream ss;
        ss << std::endl << "equals " << std::endl << m_mat;
        return ss.str();
    }

protected:
    cv::Mat m_mat;
    float_t m_epsilon;
};

// The builder function
inline MatMatcher EqualsMat(cv::Mat mat, float_t eps=std::numeric_limits<float_t>::epsilon()*100) 
{
    return MatMatcher(mat, eps);
}

