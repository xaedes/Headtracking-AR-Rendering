#include "AsusXtionSensor/AsusXtionSensor.h"
#include <OpenNI.h>
#include <cstring>

namespace ar {
namespace AsusXtionSensor {

AsusXtionSensor::AsusXtionSensor()
    : m_valid(false)
{
    openni::Status rc = openni::STATUS_OK;

    openni::Device device;
    openni::VideoStream depth, color;
    const char* deviceURI = openni::ANY_DEVICE;
    rc = openni::OpenNI::initialize();

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

    rc = m_device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
    }
    else
    {
        rc = m_depth.create(m_device, openni::SENSOR_DEPTH);
        if (rc == openni::STATUS_OK)
        {
            rc = m_depth.start();
            if (rc != openni::STATUS_OK)
            {
                printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
                m_depth.destroy();
            }
        }
        else
        {
            printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
        }

        rc = m_color.create(m_device, openni::SENSOR_COLOR);
        if (rc == openni::STATUS_OK)
        {
            rc = m_color.start();
            if (rc != openni::STATUS_OK)
            {
                printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
                m_color.destroy();
            }
        }
        else
        {
            printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
        }
    }

    if (!m_depth.isValid() || !m_color.isValid())
    {
        printf("SimpleViewer: No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
    }
    else
    {
        m_valid = true;
    }
}

AsusXtionSensor::~AsusXtionSensor()
{}

bool AsusXtionSensor::isValid()
{
    return m_valid;
}

void AsusXtionSensor::readColorFrame(cv::Mat_<cv::Vec3b>& mat)
{
    if (!m_valid) return;
    int readyStreamIndex;
    openni::VideoStream* stream = &m_color;
    openni::Status rc = openni::OpenNI::waitForAnyStream(&stream, 1, &readyStreamIndex);
    if (rc != openni::STATUS_OK || readyStreamIndex != 0)
    {
        printf("Wait failed\n");
        return;
    }
    m_color.readFrame(&m_colorFrame);
    if (m_colorFrame.isValid())
    {
        int w = m_colorFrame.getWidth();
        int h = m_colorFrame.getHeight();
        if (mat.data == 0 || mat.rows != h || mat.cols != w)
        {
            mat.create(h,w);
        }
        const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
        cv::Vec3b* pMatRow = mat[0];
        assert(sizeof(openni::RGB888Pixel) == sizeof(cv::Vec3b));
        memcpy(pMatRow, pImageRow, sizeof(openni::RGB888Pixel) * w * h);
    }
}

void AsusXtionSensor::readDepthFrame(cv::Mat_<uint16_t>& mat)
{
    if (!m_valid) return;
    int readyStreamIndex;
    openni::VideoStream* stream = &m_depth;
    openni::Status rc = openni::OpenNI::waitForAnyStream(&stream, 1, &readyStreamIndex);
    if (rc != openni::STATUS_OK || readyStreamIndex != 0)
    {
        printf("Wait failed\n");
        return;
    }
    m_depth.readFrame(&m_depthFrame);
    if (m_depthFrame.isValid())
    {
        int w = m_depthFrame.getWidth();
        int h = m_depthFrame.getHeight();
        if (mat.data == 0 || mat.rows != h || mat.cols != w)
        {
            mat.create(h,w);
        }
        const openni::DepthPixel* pImageRow = (const openni::DepthPixel*)m_depthFrame.getData();
        uint16_t* pMatRow = mat[0];
        assert(sizeof(openni::DepthPixel) == sizeof(uint16_t));
        memcpy(pMatRow, pImageRow, sizeof(openni::DepthPixel) * w * h);
    }
}

Streamer_<cv::Mat_<cv::Vec3b>> AsusXtionSensor::colorStream()
{
    return Streamer_<cv::Mat_<cv::Vec3b>>(std::bind(AsusXtionSensor::readColorFrame, this, std::placeholders::_1));
}

Streamer_<cv::Mat_<uint16_t>> AsusXtionSensor::depthStream()
{
    return Streamer_<cv::Mat_<uint16_t>>(std::bind(AsusXtionSensor::readDepthFrame, this, std::placeholders::_1));
}

} // namespace ar
} // namespace AsusXtionSensor
