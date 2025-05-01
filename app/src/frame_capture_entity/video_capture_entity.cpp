#include "video_capture_entity.hpp"

#include "logging/logger.hpp"

/**
 * @brief Конструктор
 */
VideoCaptureEntity::VideoCaptureEntity() : m_deviceId(0), m_apiId(cv::CAP_ANY)
{
    if(m_cap.open(m_deviceId, m_apiId))
    {
        spdlog::info("Find device id={}, api={}", m_deviceId, static_cast<int>(m_apiId));
    }else
    {
        //Попробовать другую камеру
        throw std::runtime_error(std::format("Cannot open device id={}, api={}", 
                                                            m_deviceId, static_cast<int>(m_apiId)));
    }
}

/**
 * @brief Выдача следующего кадра
 */
cv::Mat VideoCaptureEntity::nextFrame()
{
    cv::Mat frame;
    m_cap >> frame;
    return frame;
}
