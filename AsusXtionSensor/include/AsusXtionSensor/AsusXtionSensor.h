#pragma once

#include <inttypes.h>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include "AsusXtionSensor/Streamer.h"

namespace ar {
namespace AsusXtionSensor {

class AsusXtionSensor
{
public:
    AsusXtionSensor();
    ~AsusXtionSensor();

    bool isValid();

    void readColorFrame(cv::Mat_<cv::Vec3b>& mat);
    void readDepthFrame(cv::Mat_<uint16_t>& mat);

    Streamer_<cv::Mat_<cv::Vec3b>> colorStream();
    Streamer_<cv::Mat_<uint16_t>> depthStream();
protected:
    bool m_valid;
    openni::Device m_device;
    openni::VideoStream m_depth;
    openni::VideoStream m_color;
    openni::VideoFrameRef m_depthFrame;
    openni::VideoFrameRef m_colorFrame;
};

} // namespace AsusXtionSensor
} // namespace ar
