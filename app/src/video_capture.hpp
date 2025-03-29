#ifndef _VIDEO_CAPTURE_HPP_
#define _VIDEO_CAPTURE_HPP_

///video
#include "opencv2/videoio.hpp"

///base
#include "tasks/task.hpp"


/// @brief Класс, осуществляющий захват видео кадра и его отрисовку
class VideoCapture : public Task
{
public:

    struct CurFrameInfo : MessageBody
    {
        CurFrameInfo(std::unique_ptr<cv::Mat>&& _curFrame) : 
                            MessageBody(), 
                            curFrame(std::move(_curFrame)) 
                            {}

        std::unique_ptr<cv::Mat> curFrame;
    };


    /// @brief Конструктор
    VideoCapture();

private:

    //---------------Работа с потоком-------------------------------------------
    /**
     * @brief Основная функция задачи
     */
    void mainFunc() override;

    /**
     * @brief Принимаем сообщение от другой задачи
     * @param msg сообщение
     */
    void receiveMessage(Message &&msg) noexcept override;

    //---------------Работа с отрисовкой кадра----------------------------------
    cv::Mat m_curFrame;

    //---------------Работа с камерой-------------------------------------------
    int m_deviceId; //Id камеры
    cv::VideoCaptureAPIs m_apiId; //Id используемого API
    cv::VideoCapture m_cap;
};

#endif //_VIDEO_CAPTURE_HPP_

