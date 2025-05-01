#include "control_task.hpp"
#include "spdlog/spdlog.h"

//cv
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

//tasks interconnect
#include "shared_messages/shared_message.hpp"

///frame capturing
#include "frame_capture_entity/video_capture_entity.hpp"


//==================================================================================================
//---------------CONTROL TASK-----------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Конструктор
 * @param shapePredictorPath Путь к предиктору лиц
 */
ControlTask::ControlTask(const std::string& shapePredictorPath) : 
                        Task("ctrl", MessageDispatcher::get()),
                        m_facePredictor(shapePredictorPath)
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

    //Получаем параметры для калибровки камеры
    m_calibrator = std::make_shared<CameraCalibrator>(m_cap);

    m_gazeTracker = std::make_unique<EyeGazeTracker>(m_calibrator);
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

    // Получаем точки лица
    auto faceShape = m_facePredictor.getFaceShape(m_curFrame);

    // Если лицо найдено, рисуем точки на кадре
    if (faceShape)
    {
        drawFaceMask(*faceShape);
        drawEyeBoundingBox(*faceShape);
    }else
    {
        renderFrame("Camera", m_curFrame);
        return;
    }

    //Получаем плоские референсные точки и вычисляем позу лица
    m_gazeTracker->estimateHeadPose(m_facePredictor.getRefFacePoints());
    m_gazeTracker->update(*faceShape, m_curFrame);

    //Отправляем вектор в задачу виртуальной сцены
    if(m_gazeTracker->ready())
        sendMessage({getId(), m_virtualScene->getId(), 
            std::make_unique<GazeVectorMessage>(m_gazeTracker->getGlobalGazeVector())});

    // m_virtualScene.handleNewPoint(m_gazeTracker->getGlobalGazeVector());
    
    // if(m_virtualScene.ready())
    //     drawScene(m_virtualScene.getSceneData());

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
 * @brief Рисуем ограничивающие прямоугольники для глаз
 * @param faceShape маска лица
 */
void ControlTask::drawEyeBoundingBox(const dlib::full_object_detection& faceShape) const noexcept
{
    cv::rectangle(m_curFrame, m_facePredictor.getLeftEyeBoundingRect(faceShape), cv::Scalar(255, 0, 0), 2);
    cv::rectangle(m_curFrame, m_facePredictor.getRightEyeBoundingRect(faceShape), cv::Scalar(255, 0, 0), 2);
}

// /**
//  * @brief Отрисовываем виртуальную сцену
//  * @param sceneData сцена
//  * @param outputImage выходное изображение
//  */
// void ControlTask::drawScene(const VirtualScene::SceneData& sceneData)
// {
//     cv::Mat outputImage;
//     // Настройки
//     const int width = 800;
//     const int height = 600;
//     const cv::Scalar backgroundColor(0, 0, 0);
//     const cv::Scalar cubeColor(0, 255, 0);
//     const cv::Scalar gazeColor(255, 0, 0);
//     const cv::Scalar intersectionColor(255, 255, 255);

//     // Создаем черное изображение
//     outputImage = cv::Mat(height, width, CV_8UC3, backgroundColor);

//     // Простейшая проекция 3D->2D: просто X,Y + масштабирование
//     auto project = [width, height](const cv::Point3d& pt) -> cv::Point {
//         const double scale = 200.0; // масштаб для видимости
//         int x = static_cast<int>(width / 2 + pt.x * scale);
//         int y = static_cast<int>(height / 2 - pt.y * scale);
//         return cv::Point(x, y);
//     };

//     // Рисуем параллелепипед (ребра)
//     const std::vector<std::pair<int, int>> edges = {
//         {0,1}, {1,2}, {2,3}, {3,0}, // передняя грань
//         {4,5}, {5,6}, {6,7}, {7,4}, // задняя грань
//         {0,4}, {1,5}, {2,6}, {3,7}  // боковые ребра
//     };

//     for (const auto& edge : edges) {
//         cv::line(outputImage,
//                  project(sceneData.cubeVertices[edge.first]),
//                  project(sceneData.cubeVertices[edge.second]),
//                  cubeColor, 2);
//     }

//     // Рисуем вектор взгляда
//     cv::Point gazeStart = project(cv::Point3d(0,0,0));
//     cv::Point gazeEnd = project(cv::Point3d(sceneData.gazeVector * 2.0)); // удлинён для наглядности
//     cv::arrowedLine(outputImage, gazeStart, gazeEnd, gazeColor, 2);

//     // Рисуем точку пересечения
//     if (sceneData.lastIntersectionPoint) {
//         cv::circle(outputImage, project(*sceneData.lastIntersectionPoint), 6, intersectionColor, cv::FILLED);
//     }

//     //Рендерим картинку
//     renderFrame("Virtual scene", outputImage);
// }

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
