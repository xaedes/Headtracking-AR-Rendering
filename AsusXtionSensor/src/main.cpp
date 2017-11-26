#include <iostream>

#include "AsusXtionSensor/version.h"
#include "AsusXtionSensor/AsusXtionSensor.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 

using namespace ar::AsusXtionSensor;

int main(int argc, char* argv[])
{   
    std::cout << "version " << Version::getString() << std::endl;
    std::cout << "revision " << Version::getRevision() << std::endl;

    AsusXtionSensor sensor;
    Streamer_<cv::Mat_<cv::Vec3b>> color = sensor.colorStream();
    Streamer_<cv::Mat_<uint16_t>> depth = sensor.depthStream();

    int key = 255;
    cv::Mat_<cv::Vec3b> colorFrame;
    cv::Mat_<uint16_t> depthFrame;
    
    std::cout << "key\t" << (key & 0xff) << std::endl;
    while(1)
    {
        color >> colorFrame;
        depth >> depthFrame;
        if(colorFrame.data)
        {
            cv::imshow("color", colorFrame);
        }
        if(depthFrame.data)
        {
            cv::Mat_<float> depthFloat(depthFrame);
            depthFloat = 1-(depthFloat / (float)0xffff);
            cv::imshow("depth", depthFloat);
        }
        key = cv::waitKey(10);
        // std::cout << "key\t" << (key & 0xff) << std::endl;
    }
}
