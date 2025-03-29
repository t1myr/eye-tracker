#include "video_capture.hpp"
#include "spdlog/spdlog.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "dlib/opencv.h"


//==================================================================================================
//---------------VIDEO CAPTURE----------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Конструктор
 * @param shapePredictorPath Путь к предиктору лиц
 */
VideoCapture::VideoCapture(const std::string& shapePredictorPath) : 
                        Task("video", MessageDispatcher::get()), 
                        m_deviceId(0),
                        m_apiId(cv::CAP_ANY),
                        m_facePredictor(shapePredictorPath)
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

    m_cap.read(m_curFrame);

    if(m_curFrame.empty())
    {
        spdlog::warn("Read empty frame from capture!!!");
        return;
    }

    dlib::array2d<dlib::rgb_pixel> dlibFrame;
    dlib::assign_image(dlibFrame, dlib::cv_image<dlib::rgb_pixel>(m_curFrame));

    // Получаем точки лица
    auto faceShape = m_facePredictor.getFaceShape(dlibFrame);

    // Если лицо найдено, рисуем точки на кадре
    if (faceShape)
    {
        for (int i = 0; i < faceShape->num_parts(); ++i)
        {
            const auto& p = faceShape->part(i);
            cv::circle(m_curFrame, cv::Point(p.x(), p.y()), 2, cv::Scalar(0, 255, 0), -1);
        }
    }

    //Рендерим картинку
    cv::imshow("Camera", m_curFrame);
    cv::waitKey(1);
}
