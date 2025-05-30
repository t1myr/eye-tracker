#include "control_task.hpp"

//cv
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

//logger
#include "logging/logger.hpp"

//tasks interconnect
#include "shared_messages/shared_message.hpp"

///frame capturing
#include "frame_capture_entity/video_capture_entity.hpp"
#include "frame_capture_entity/picture_capture_entity.hpp"


//==================================================================================================
//---------------CONTROL TASK-----------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Конструктор
 * @param shapePredictorPath Путь к предиктору лиц
 */
ControlTask::ControlTask(const std::string& shapePredictorPath) : 
    Task("ctrl", MessageDispatcher::get()),
    m_shapePredictorPath(shapePredictorPath)
{

}

//==================================================================================================
//---------------Работа с потоком-------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Инициализация
 */
void ControlTask::init()
{
    m_cap = std::make_shared<VideoCaptureEntity>();

    auto calibCapture = std::make_shared<PictureCaptureEntity>("C:\\Users\\timur\\Projects\\eye-tracker\\data\\calibrate");

    //Получаем параметры для калибровки камеры
    m_calibrator = std::make_shared<CameraCalibrator>(calibCapture);

    m_globalGazeEstimator = std::make_unique<GlobalGazeEstimator>(m_calibrator, m_shapePredictorPath);
}

/**
 * @brief Основная функция задачи
 */
void ControlTask::mainFunc()
{
    // Читаем новый кадр
    m_curFrame = m_cap->nextFrame();

    // Если кадра нет, дальше ничего не делаем
    if(m_curFrame.empty())
    {
        spdlog::warn("Read empty frame from capture!!!");
        return;
    }

    //Вычисляем вектор взгляда
    auto gazeVec = m_globalGazeEstimator->estimate(m_curFrame);

    if(gazeVec)
    {
        //TODO send gazeVec to virtual Scene
    }

    renderFrame("Camera", m_curFrame);
}

//==================================================================================================
//---------------Работа с сообщениями---------------------------------------------------------------
//==================================================================================================
/**
 * @brief Принимаем сообщение от другой задачи
 * @param msg сообщение
 */
void ControlTask::receiveMessage(Message &&msg) 
{
    if(msg.src == m_virtualScene->getId())
    {
        auto sharedMsg = dynamic_cast<SharedMessage*>(msg.body.get());

        switch(sharedMsg->getType())
        {
            case SharedMessage::kFrame:{
                auto frameMsg = dynamic_cast<FrameMessage*>(msg.body.get());
                renderFrame("Virtual Scene", frameMsg->getFrame());
                break;
            }
            default:{
                throw std::invalid_argument(std::format("Receive unknown mt={}, src={}, dst={}", 
                    sharedMsg->getType(), msg.src, msg.dst));
                break;
            }
        }
    }else
    {
        Task::receiveMessage(std::move(msg));
    }
}

//==================================================================================================
//---------------Работа с отрисовкой кадра----------------------------------------------------------
//==================================================================================================
/**
 * @brief Рисуем маску лица
 * @param faceShape маска лица
 */
void ControlTask::drawFaceMask(const dlib::full_object_detection& faceShape) const noexcept
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
 * @brief Отрисовка текущего кадра
 * @param frameName имя кадра
 * @param frame кадр
 */
void ControlTask::renderFrame(const std::string& frameName, const cv::Mat& frame)
{
    cv::imshow(frameName, frame);
    cv::waitKey(1);
}
