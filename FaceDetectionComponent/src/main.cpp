#include <iostream>

#include "FaceDetectionComponent/version.h"
#include "AsusXtionSensor/AsusXtionSensor.h"
#include "FaceDetectionComponent/FaceDetectionComponent.h"

#include <chrono>
#include <thread>

using namespace ar::FaceDetectionComponent;
using namespace ar::AsusXtionSensor;

wait(double seconds)
{
    std::this_thread::sleep_for(
        std::chrono::duration<double>(seconds)
    );
}

int main(int argc, char* argv[])
{
	std::cout << "version " << Version::getString() << std::endl;
	std::cout << "revision " << Version::getRevision() << std::endl;

    AsusXtionSensor sensor;
    Streamer_<cv::Mat_<cv::Vec3b>> color = sensor.colorStream();
    // Streamer_<cv::Mat_<uint16_t>> depth = sensor.depthStream();
    cv::Mat_<cv::Vec3b> colorFrame;
    cv::Mat_<cv::Vec3b> colorFrameScaled;
    // cv::Mat_<uint16_t> depthFrame;

    FaceDetectionComponent faceDetection;
    faceDetection.start(argc,argv);
    double fps = 60;
    while(1)
    {
        color >> colorFrame;

        cv::Size size = colorFrame.size();
        cv::Size scaledSize((int)ceil(size.width * 1.5), (int)ceil(size.height * 1.5));
        cv::resize(colorFrame, colorFrameScaled, scaledSize);

        // depth >> depthFrame;
        faceDetection.pushImage(colorFrameScaled);
        std::cout << "time \t" << faceDetection.timeSinceStart() << std::endl;

        cv::Mat visualizedImage = faceDetection.visualizedImage(colorFrameScaled);
        cv::Vec6d headPose = faceDetection.headPose();
        if(colorFrameScaled.data)
        {
            cv::imshow("colorFrameScaled", colorFrameScaled);
        }
        if(visualizedImage.data)
        {
            cv::imshow("visualized", visualizedImage);
        }
        int key = cv::waitKey((int)(1000/fps));
        // wait(1/fps);
    }
    faceDetection.stop();

}
