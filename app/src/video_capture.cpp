#include "video_capture.hpp"
#include "spdlog/spdlog.h"

//cv
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
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
    auto renderFrame = [this]()
    {
        //Рендерим картинку
        cv::imshow("Camera", m_curFrame);
        cv::waitKey(1);
    };

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
        drawFaceMask(*faceShape);
        drawEyeBoundingBox(*faceShape);
    }else
    {
        renderFrame();
        return;
    }
    cv::Vec3d rvec, tvec;

    //Вычисляем позу лица
    bool success = cv::solvePnP(
        m_3dmodelPoints, m_facePredictor.getRefFacePoints(),
        m_calibrator->getCameraMatrix(), m_calibrator->getDistCoeffs(),
        rvec, tvec
    );

    if(!success)
    {
        spdlog::warn("Head pose estimation failed");
        renderFrame();
    }

    cv::Matx33d rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);

    m_gazeTracker.setHeadPose(rotationMatrix, tvec);

    m_gazeTracker.update(*faceShape, m_curFrame);

    if(m_gazeTracker.getGlobalGazeVector().has_value())
    {
        spdlog::info("Calculated gaze vector!!!!");
    }

    renderFrame();
}

/**
 * @brief Инициализация
 */
void VideoCapture::init()
{
    //Получаем параметры для калибровки камеры
    m_calibrator = std::make_unique<CameraCalibrator>(m_cap);
}

//==================================================================================================
//---------------Работа с отрисовкой кадра----------------------------------------------------------
//==================================================================================================
/**
 * @brief Рисуем маску лица
 * @param faceShape маска лица
 */
void VideoCapture::drawFaceMask(const dlib::full_object_detection& faceShape) const noexcept
{
    for (int i = 0; i < faceShape.num_parts(); ++i)
    {
        cv::Scalar color;
        const auto& p = faceShape.part(i);
        if(FaceShapePredictor::kLeftEyeStartPoint <= i && i <= FaceShapePredictor::kLeftEyeEndPoint)
        {
            color = cv::Scalar(240, 255, 255);
        }else if(FaceShapePredictor::kLeftEyeStartPoint <= i && i <= FaceShapePredictor::kRightEyeEndPoint)
        {
            color = cv::Scalar(242, 44, 54);
        }else
        {
            color = cv::Scalar(0, 255, 0);
        }

        cv::circle(m_curFrame, cv::Point(p.x(), p.y()), 2, color, -1);
    }
}

/**
 * @brief Рисуем ограничивающие прямоугольники для глаз
 * @param faceShape маска лица
 */
void VideoCapture::drawEyeBoundingBox(const dlib::full_object_detection& faceShape) const noexcept
{
    cv::rectangle(m_curFrame, m_facePredictor.getLeftEyeBoundingRect(faceShape), cv::Scalar(255, 0, 0), 2);
    cv::rectangle(m_curFrame, m_facePredictor.getRightEyeBoundingRect(faceShape), cv::Scalar(255, 0, 0), 2);
}
