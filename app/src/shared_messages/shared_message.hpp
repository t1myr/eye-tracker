#ifndef _SHARED_MESSAGE_HPP_
#define _SHARED_MESSAGE_HPP_

#include "opencv2/core.hpp"

#include "../tasks/message_dispatcher.hpp"

/// @brief Общее сообщение
class SharedMessage : public MessageBody
{
public:

    enum Type : uint8_t
    {
        kUnknown = 0,
        kGazeVector,
        kFrame
    };

    SharedMessage(Type type) : type(type) {}

    uint8_t getType() const { return type; }
private:
    Type type{kUnknown};
};


/// @brief Сообщение о новом фрейме
class FrameMessage : public SharedMessage
{
public:
    FrameMessage(const cv::Mat& frame) : SharedMessage(Type::kFrame), frame(frame) {}

    const cv::Mat& getFrame() const noexcept { return frame; }
private:
    cv::Mat frame;
};

/// @brief Сообщение о векторе
class GazeVectorMessage : public SharedMessage
{
public:
    GazeVectorMessage(const cv::Vec3d& vector) : SharedMessage(Type::kGazeVector), gazeVector(vector) {}

    const cv::Vec3d& getVector() const noexcept { return gazeVector; }
private:
    cv::Vec3d gazeVector;
};

#endif // _SHARED_MESSAGE_HPP_