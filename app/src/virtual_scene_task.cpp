#include "virtual_scene_task.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <numeric>

#include "shared_messages/shared_message.hpp"

/**
 * @brief Конструктор
 */
VirtualScene::VirtualScene() : Task("v_scene", MessageDispatcher::get())
{

}

void VirtualScene::handleNewPoint(const cv::Vec3d& gazeVector)
{
    if (!m_ready) {
        // Накапливаем калибровочные точки
        m_points.push_back(gazeVector);
        if (m_points.size() >= kPointsForSurface) {
            buildSurface();
            m_ready = true;
        }
        return;
    }

    // Сохраняем последний взгляд
    m_lastGaze = gazeVector;
    m_lastIntersection = intersectRayWithPlane(gazeVector);
    
}

VirtualScene::SceneData VirtualScene::getSceneData() const
{
    SceneData data;
    data.cubeVertices = m_cubeVertices;
    data.gazeVector = m_lastGaze;
    data.lastIntersectionPoint = m_lastIntersection;

    return data;
}

//==================================================================================================
//---------------Работа с потоком-------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Инициализация в потоке
 */
void VirtualScene::init()
{
    
}

/**
 * @brief Основная функция задачи
 */
void VirtualScene::mainFunc()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

//==================================================================================================
//---------------Работа с сообщениями---------------------------------------------------------------
//==================================================================================================
/**
 * @brief Принимаем сообщение от другой задачи
 * @param msg сообщение
 */
void VirtualScene::receiveMessage(Message &&msg) 
{
    if(msg.src == m_ctrl->getId())
    {
        auto sharedMsg = dynamic_cast<SharedMessage*>(msg.body.get());
        switch(sharedMsg->getType())
        {
            case SharedMessage::kGazeVector:{
                auto gazeMsg = dynamic_cast<GazeVectorMessage*>(msg.body.get());
                handleNewPoint(gazeMsg->getVector());
                break;
            }
            default:{
                throw std::invalid_argument(std::format("Receive unknown mt={}, src={}, dst={}", sharedMsg->getType(), msg.src, msg.dst));
                break;
            }
        }
    }else
    {
        Task::receiveMessage(std::move(msg));
    }
}

void VirtualScene::buildSurface()
{
    // Вычисляем центр всех калибровочных точек
    cv::Vec3d center(0, 0, 0);
    for (const auto& pt : m_points) {
        center += pt;
    }
    center /= static_cast<double>(m_points.size());

    // Оценка нормали плоскости (наивная) через PCA
    cv::Mat dataPts(m_points.size(), 3, CV_64F);
    for (size_t i = 0; i < m_points.size(); ++i) {
        dataPts.at<double>(i, 0) = m_points[i][0] - center[0];
        dataPts.at<double>(i, 1) = m_points[i][1] - center[1];
        dataPts.at<double>(i, 2) = m_points[i][2] - center[2];
    }

    cv::PCA pca(dataPts, cv::Mat(), cv::PCA::DATA_AS_ROW);
    m_planeNormal = cv::normalize(cv::Vec3d(
        pca.eigenvectors.at<double>(2, 0),
        pca.eigenvectors.at<double>(2, 1),
        pca.eigenvectors.at<double>(2, 2)
    ));
    m_planeD = -m_planeNormal.dot(center);

    // Строим куб вокруг центра
    double size = 0.2; // Параллелепипед размером 20см
    cv::Vec3d right = cv::normalize(cv::Vec3d(1, 0, 0).cross(m_planeNormal));
    cv::Vec3d up = cv::normalize(m_planeNormal.cross(right));

    m_cubeVertices.clear();
    m_cubeVertices.push_back(center - right * size - up * size); // левый нижний
    m_cubeVertices.push_back(center + right * size - up * size); // правый нижний
    m_cubeVertices.push_back(center + right * size + up * size); // правый верхний
    m_cubeVertices.push_back(center - right * size + up * size); // левый верхний

    // Задняя плоскость параллелепипеда (чуть смещённая по нормали)
    cv::Point3d backOffset = m_planeNormal * (-0.1); // 10 см глубина
    m_cubeVertices.push_back(m_cubeVertices[0] + backOffset);
    m_cubeVertices.push_back(m_cubeVertices[1] + backOffset);
    m_cubeVertices.push_back(m_cubeVertices[2] + backOffset);
    m_cubeVertices.push_back(m_cubeVertices[3] + backOffset);
}

std::optional<cv::Point3d> VirtualScene::intersectRayWithPlane(const cv::Vec3d& ray) const
{
    double denom = m_planeNormal.dot(ray);
    if (std::abs(denom) < 1e-6) {
        return std::nullopt; // Параллельно плоскости
    }
    double t = -(m_planeD) / denom;
    if (t < 0) {
        return std::nullopt; // Пересечение позади камеры
    }
    return cv::Point3d(ray * t);
}

/**
 * @brief Генерируем кадр виртуальной сцены
 */
cv::Mat VirtualScene::getSceneFrame(const VirtualScene::SceneData& sceneData)
{
    cv::Mat outputImage;
    // Настройки
    const int width = 800;
    const int height = 600;
    const cv::Scalar backgroundColor(0, 0, 0);
    const cv::Scalar cubeColor(0, 255, 0);
    const cv::Scalar gazeColor(255, 0, 0);
    const cv::Scalar intersectionColor(255, 255, 255);

    // Создаем черное изображение
    outputImage = cv::Mat(height, width, CV_8UC3, backgroundColor);

    // Простейшая проекция 3D->2D: просто X,Y + масштабирование
    auto project = [width, height](const cv::Point3d& pt) -> cv::Point {
        const double scale = 200.0; // масштаб для видимости
        int x = static_cast<int>(width / 2 + pt.x * scale);
        int y = static_cast<int>(height / 2 - pt.y * scale);
        return cv::Point(x, y);
    };

    // Рисуем параллелепипед (ребра)
    const std::vector<std::pair<int, int>> edges = {
        {0,1}, {1,2}, {2,3}, {3,0}, // передняя грань
        {4,5}, {5,6}, {6,7}, {7,4}, // задняя грань
        {0,4}, {1,5}, {2,6}, {3,7}  // боковые ребра
    };

    for (const auto& edge : edges) {
        cv::line(outputImage,
                 project(sceneData.cubeVertices[edge.first]),
                 project(sceneData.cubeVertices[edge.second]),
                 cubeColor, 2);
    }

    // Рисуем вектор взгляда
    cv::Point gazeStart = project(cv::Point3d(0,0,0));
    cv::Point gazeEnd = project(cv::Point3d(sceneData.gazeVector * 2.0)); // удлинён для наглядности
    cv::arrowedLine(outputImage, gazeStart, gazeEnd, gazeColor, 2);

    // Рисуем точку пересечения
    if (sceneData.lastIntersectionPoint) {
        cv::circle(outputImage, project(*sceneData.lastIntersectionPoint), 6, intersectionColor, cv::FILLED);
    }

    return outputImage;
}
