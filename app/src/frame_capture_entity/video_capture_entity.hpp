#ifndef _VIDEO_CAPTURE_ENTITY_HPP_
#define _VIDEO_CAPTURE_ENTITY_HPP_

///video
#include "opencv2/videoio.hpp"

//base
#include "frame_capture_entity.hpp"

/// @brief Захват кадра с видеопотока
class VideoCaptureEntity : public FrameCaptureEntity
{
public:
    /// @brief Конструктор
    VideoCaptureEntity();

    /// @brief Выдача следующего кадра
    cv::Mat nextFrame() override;

private:
    int m_deviceId; //Id камеры
    cv::VideoCaptureAPIs m_apiId; //Id используемого API
    cv::VideoCapture m_cap;
};

#endif // _VIDEO_CAPTURE_ENTITY_HPP_
