#include "video_capture.hpp"
#include "spdlog/spdlog.h"
#include "opencv2/highgui.hpp"


VideoCapture::VideoCapture() : m_deviceId(0), m_apiId(cv::CAP_ANY)
{
    spdlog::info("Video capture created");
}

VideoCapture::~VideoCapture()
{
    spdlog::info("Video capture destroyed");
}

void VideoCapture::start()
{
    spdlog::info("Starting capture...");
    cv::Mat frame;

    //infinite loop
    for(;;)
    {  
        m_cap.read(frame);
        if(frame.empty())
        {
            spdlog::critical("Empty frame, capture corrupted");
            break;
        }
        cv::imshow("Camera", frame);
        auto key = cv::waitKey(5);
        if( key >= 0 || cv::getWindowProperty("Camera", cv::WND_PROP_VISIBLE) < 1)
            break;
    }
    spdlog::info("Capture ended");
}

bool VideoCapture::open()
{
    if(m_cap.open(m_deviceId, m_apiId))
    {
        spdlog::info("Video capture opened successfully");
        return true;
    }
    return false;
}
