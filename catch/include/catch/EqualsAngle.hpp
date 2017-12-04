
#include <opencv2/opencv.hpp>
#include "catch/catch.hpp"
#define PI           3.14159265358979323846


// The matcher class
class AngleMatcher : public Catch::MatcherBase<float> 
{
public:
    AngleMatcher(float_t angle, float_t eps=1e-3) 
        : m_angle(angle)
        , m_epsilon(eps)
    {}

    AngleMatcher epsilon(float_t eps)
    {
        return AngleMatcher(m_angle, eps);
    }

    // Performs the test for this matcher
    virtual bool match( float_t const& otherAngle ) const override {
        float_t abs_diff = abs(otherAngle - m_angle);
        float_t multiples_of_2pi = abs_diff / (2*PI);
        int int_multiples_of_2pi = (int)multiples_of_2pi;
        return (abs(multiples_of_2pi - int_multiples_of_2pi) < m_epsilon);
    }

    // Produces a string describing what this matcher does. It should
    // include any provided data (the begin/ end in this case) and
    // be written as if it were stating a fact (in the output it will be
    // preceded by the value under test).
    virtual std::string describe() const {
        std::ostringstream ss;
        ss << std::endl << "equals " << std::endl << m_angle;
        return ss.str();
    }

protected:
    float_t m_angle;
    float_t m_epsilon;
};

// The builder function
inline AngleMatcher EqualsAngle(float_t angle, float_t eps=std::numeric_limits<float_t>::epsilon()*100) 
{
    return AngleMatcher(angle, eps);
}

