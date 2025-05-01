#ifndef _FRAME_CAPTURE_ENTITY_HPP_
#define _FRAME_CAPTURE_ENTITY_HPP_

#include "opencv2/core.hpp"

/// @brief Захват кадра
class FrameCaptureEntity
{
public:
    /// @brief Конструктор
    FrameCaptureEntity() = default;

    /// @brief Выдача следующего кадра
    virtual cv::Mat nextFrame() = 0;
};

#endif // _FRAME_CAPTURE_ENTITY_HPP_
