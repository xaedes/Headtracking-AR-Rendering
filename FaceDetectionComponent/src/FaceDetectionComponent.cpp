#include "FaceDetectionComponent/FaceDetectionComponent.h"

#include "FaceDetection/FaceDetection.h"

#include "common/get_arguments.h"

using ar::common::get_arguments;

namespace ar {
namespace FaceDetectionComponent {


FaceDetectionComponent::~FaceDetectionComponent()
{

}

FaceDetectionComponent::FaceDetectionComponent()
{
}

void FaceDetectionComponent::stop()
{
    if(m_running)
    {
        m_running = false;
        m_faceDetectionThread.join();
    }
}
void FaceDetectionComponent::start(int argc, char* argv[])
{
    startTime();
    m_argc = argc;
    m_argv = argv;
    m_faceDetectionThread = std::thread(
        &FaceDetectionComponent::runFaceDetection,
        this
    );
}

void FaceDetectionComponent::pushImage(const cv::Mat_<cv::Vec3b>& image)
{
    std::unique_lock<std::mutex> lock(m_imageQueueMutex);
    // std::cout << "push image" << std::endl;
    m_imageQueue.push(make_pair(timeSinceStart(), image));
    while(m_imageQueue.size() > 5)
    {
        m_imageQueue.pop();
    }    
    // m_imageQueue.push(make_pair(timeSinceStart(), image.clone()));
    // std::cout << "queue size " << m_imageQueue.size()  << std::endl;
}

void FaceDetectionComponent::resetObservationFilterIfNan(const FaceDetectionFilter::FaceDetectionFilterParameters& faceDetectionFilterParameters)
{
    cv::Vec6d filteredHeadPose = m_faceDetectionObservationFilter.headPose();
    if(std::isnan(filteredHeadPose[0]) 
        || std::isnan(filteredHeadPose[1]) 
        || std::isnan(filteredHeadPose[2]) 
        || std::isnan(filteredHeadPose[3]) 
        || std::isnan(filteredHeadPose[4]) 
        || std::isnan(filteredHeadPose[5]) )
    {
        m_faceDetectionObservationFilter = FaceDetectionFilter::FaceDetectionFilter(faceDetectionFilterParameters);
    }
}

void FaceDetectionComponent::runFaceDetection()
{
    m_faceDetection.reset(new FaceDetection::FaceDetection(m_argc, m_argv));

    FaceDetectionFilter::FaceDetectionFilterParameters faceDetectionFilterParameters;
    auto arguments = get_arguments(m_argc, m_argv);
    faceDetectionFilterParameters.applyArguments(arguments);
    faceDetectionFilterParameters.print();
    m_faceDetectionObservationFilter = FaceDetectionFilter::FaceDetectionFilter(faceDetectionFilterParameters);
    updatePredictionFilter();

    m_running = true;
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    while(m_running)
    {
        double imageTime;
        cv::Mat_<cv::Vec3b> image;
        // read image from queue
        {
            // std::cout << "get image" << std::endl;
            std::unique_lock<std::mutex> lock(m_imageQueueMutex);
            // std::cout << "queue size " << m_imageQueue.size()  << std::endl;
            if(m_imageQueue.empty())
            {
                // std::cout << "queue empty" << std::endl;
                wait(0.001);
                continue;
            }
            const std::pair<double,cv::Mat_<cv::Vec3b>>& imageItem = m_imageQueue.front();
            imageTime = imageItem.first;
            image = imageItem.second;
            // std::cout << "got image" << std::endl;
            m_imageQueue.pop();
            while(m_imageQueue.size() > 5)
            {
                m_imageQueue.pop();
            }
        }

        // std::cout << "imageTime " << imageTime << std::endl;
        // process 
        if (image.data && imageTime >= m_faceDetectionObservationFilter.getFilterTime())
        {
            // std::cout << "process image" << std::endl;
            m_faceDetection->feedColorFrame(image);
            cv::Vec6d headPose = m_faceDetection->headPose();
            // std::cout << "headPose \t" << headPose << std::endl;
            double detectionCertainty = m_faceDetection->detectionCertainty();
            // std::cout << "detectionCertainty \t" << detectionCertainty << std::endl;
            // std::cout << "imageTime \t" << imageTime << std::endl;
            // std::cout << "m_faceDetectionObservationFilter.getFilterTime() \t" << m_faceDetectionObservationFilter.getFilterTime() << std::endl;
            // std::cout << "dt \t" << imageTime-m_faceDetectionObservationFilter.getFilterTime() << std::endl;
            
            resetObservationFilterIfNan(faceDetectionFilterParameters);

    // std::unique_lock<std::mutex> lock(m_faceDetectionPredictionFilterMutex);
            m_faceDetectionObservationFilter.predict(imageTime);

            // for(int i=0;i<6;i++)
            // {
                // std::cout << "m_faceDetectionObservationFilter.filter(i).print();" << std::endl;
            //     m_faceDetectionObservationFilter.filter(i).print();
            // }
            // std::cout << "m_faceDetectionObservationFilter.predict(imageTime) \t"<< m_faceDetectionObservationFilter.predict(imageTime) << std::endl;
            m_faceDetectionObservationFilter.update(imageTime, headPose, detectionCertainty);
            // std::cout << "m_faceDetectionObservationFilter.update(imageTime, headPose, detectionCertainty) \t" << m_faceDetectionObservationFilter.update(imageTime, headPose, detectionCertainty) << std::endl;
            // std::cout << "m_faceDetectionObservationFilter.headPose() \t" << m_faceDetectionObservationFilter.headPose() << std::endl;

            updatePredictionFilter();

            {
                std::unique_lock<std::mutex> lock(m_lastImageMutex);
                if(!m_lastImage.data)
                {
                    m_lastImage = image.clone();
                }
                else
                {
                    image.copyTo(m_lastImage);
                }
        // std::cout << "m_lastImage.rows " << m_lastImage.rows << std::endl;
        // std::cout << "m_lastImage.cols " << m_lastImage.cols << std::endl;
            }
        }
    }
}


void FaceDetectionComponent::updatePredictionFilter()
{
    std::unique_lock<std::mutex> lock(m_faceDetectionPredictionFilterMutex);
    m_faceDetectionPredictionFilter = m_faceDetectionObservationFilter;
    // std::cout << "m_faceDetectionPredictionFilter.headPose() \t" << m_faceDetectionPredictionFilter.headPose() << std::endl;
}


cv::Mat FaceDetectionComponent::visualizedImage()
{
    cv::Mat vis;
    {
        std::unique_lock<std::mutex> lock(m_lastImageMutex);
        vis = visualizedImage(m_lastImage);
    }
    return vis;
}

cv::Mat FaceDetectionComponent::visualizedImage(const cv::Mat_<cv::Vec3b>& colorFrame)
{
    cv::Vec6d headPose = this->headPose();
    return visualizeTracking(colorFrame, headPose);;
}

cv::Vec6d FaceDetectionComponent::headPose()
{
    std::unique_lock<std::mutex> lock(m_faceDetectionPredictionFilterMutex);
    m_faceDetectionPredictionFilter.predict(timeSinceStart());
    // std::cout << "m_faceDetectionPredictionFilter.headPose() \t" << m_faceDetectionPredictionFilter.headPose() << std::endl;
    return m_faceDetectionPredictionFilter.headPose();
    // return m_faceDetection->headPose();
}


void FaceDetectionComponent::startTime()
{
    m_startTime = std::chrono::system_clock::now();
}

double FaceDetectionComponent::timeSinceStart() const
{
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - m_startTime;
    return diff.count();
}

void FaceDetectionComponent::wait(double seconds)
{
    std::this_thread::sleep_for(
        std::chrono::duration<double>(seconds)
    );
}

cv::Mat FaceDetectionComponent::visualizeTracking(
    const cv::Mat_<cv::Vec3b>& colorFrame, 
    cv::Vec6d headPose
) 
{
    double fx;
    double fy; 
    double cx; 
    double cy;
    cv::Mat visualization;

    cx = colorFrame.cols / 2.0f;
    cy = colorFrame.rows / 2.0f;

    fx = 500 * (colorFrame.cols / 640.0);
    fy = 500 * (colorFrame.rows / 480.0);

    fx = (fx + fy) / 2.0;
    fy = fx;

        // std::cout << "colorFrame.rows " << colorFrame.rows << std::endl;
        // std::cout << "colorFrame.cols " << colorFrame.cols << std::endl;
    // visualization = colorFrame.clone();
    visualization = colorFrame;
    // Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
    // double detection_certainty = faceModel.detection_certainty;
    // bool detection_success = faceModel.detection_success;

    // double visualisation_boundary = 0.2;

    // Only draw if the reliability is reasonable, the value is slightly ad-hoc
    // if (detection_certainty < visualisation_boundary)
    // {

        // double vis_certainty = detection_certainty;
        // if (vis_certainty > 1)
        //     vis_certainty = 1;
        // if (vis_certainty < -1)
        //     vis_certainty = -1;

        // double vis_certainty = 0;

        // vis_certainty = (vis_certainty + 1) / (visualisation_boundary + 1);

        // A rough heuristic for box around the face width
        int thickness = (int)std::ceil(2.0* ((double)visualization.cols) / 640.0);

        // std::cout << "visualize headPose " << headPose << std::endl;
        // Draw it in reddish if uncertain, blueish if certain
        LandmarkDetector::DrawBox(visualization, headPose, cv::Scalar(255, 255, 0), thickness, fx, fy, cx, cy);

    // }

    // // Work out the framerate
    // if (m_frameCount % 10 == 0)
    // {
    //     double t1 = cv::getTickCount();
    //     m_fpsTracker = 10.0 / (double(t1 - m_t0) / cv::getTickFrequency());
    //     m_t0 = t1;
    // }

    // // Write out the framerate on the image before displaying it
    // string fpsSt("FPS:" + std::to_string((int)m_fpsTracker));
    // cv::putText(visualization, fpsSt, cv::Point(10, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1, CV_AA);

    // if (!m_faceModelParameters.quiet_mode)
    // {
    //     cv::namedWindow("tracking_result", 1);
    //     cv::imshow("tracking_result", visualization);
    // }
        // std::cout << "visualization.rows " << visualization.rows << std::endl;
        // std::cout << "visualization.cols " << visualization.cols << std::endl;
    
    return visualization;
}



} // namespace FaceDetectionComponent
} // namespace ar
