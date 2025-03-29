#include "video_capture.hpp"
#include "spdlog/spdlog.h"
#include "opencv2/highgui.hpp"

//==================================================================================================
//---------------VIDEO CAPTURE----------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Конструктор
 */
VideoCapture::VideoCapture() : Task("video", MessageDispatcher::get()), m_deviceId(0), m_apiId(cv::CAP_ANY)
{
    if(m_cap.open(m_deviceId, m_apiId))
    {
        spdlog::info("Find device id={}, api={}", m_deviceId, static_cast<int>(m_apiId));
    }else
    {
        //Попробовать другую камеру
        spdlog::warn("Cannot open device id={}, api={}, try next", m_deviceId, static_cast<int>(m_apiId));
    }
}

//==================================================================================================
//---------------Работа с потоком-------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Основная функция задачи
 */
void VideoCapture::mainFunc()
{
    if(!m_cap.isOpened())
        return;

    auto curFrame = std::make_unique<cv::Mat>();

    m_cap.read(*curFrame);

    if(curFrame->empty())
    {
        spdlog::warn("Read empty frame from capture!!!");
    }
    //Рендерим картинку
    cv::imshow("Camera", *curFrame);
    cv::waitKey(1);
}


void VideoCapture::receiveMessage(Message &&msg) noexcept
{

}
