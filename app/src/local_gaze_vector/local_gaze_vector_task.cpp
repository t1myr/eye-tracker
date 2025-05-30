#include "local_gaze_vector_task.hpp"

#include "utils/assert.hpp"
#include "utils/cast.hpp"


/// @brief Конструктор
LocalGazeVectorTask::LocalGazeVectorTask() : 
    PackageTask("loc_gaze", MessageDispatcher::get()),
    m_threadPool(ThreadPool::get())
{

}

/// @brief Запускаем расчет задачи
std::unique_ptr<PackageTask::ExecuteBody> LocalGazeVectorTask::execute(
    std::unique_ptr<ExecuteBody>&& body)
{
    //Проверяем переданную область
    auto payload = dynamicCastPtr(Body, body.get());

    auto computeEllipse = [this](const cv::Mat& roi) -> std::optional<cv::RotatedRect> {
        return m_ellipseFitter.fit(roi);
    };

    //Вычислить эллипс
    auto leftEyeEllipseFuture = m_threadPool->enqueue(computeEllipse, payload->left.roi);
    auto rightEyeEllipseFuture = m_threadPool->enqueue(computeEllipse, payload->right.roi);

    auto leftEyeEllipse = leftEyeEllipseFuture.get();
    auto rightEyeEllipse = rightEyeEllipseFuture.get();

    if(leftEyeEllipse.has_value() && rightEyeEllipse.has_value())
    {
        //Создаем результат
        auto result = std::make_unique<Result>();
        result->left = computeLocalGaze(leftEyeEllipse->center, payload->left.eyePoints);
        result->right = computeLocalGaze(rightEyeEllipse->center, payload->right.eyePoints);
        return result;
    }

    return nullptr;
}

/// @brief Вычисляем локальный вектор взгляда
LocalGazeVectorTask::EyeEntity LocalGazeVectorTask::computeLocalGaze(
    const cv::Point2d& irisCenter, 
    const std::vector<cv::Point2d>& points) const
{
    EyeEntity entity;
    entity.eyeCenter = getEyeCenter(points);
    entity.gazeVec = getLocalGazeVector(entity.eyeCenter, 
                                        irisCenter, 
                                        getEyeRadius(entity.eyeCenter, points.front()) //Берем в качестве угловой точки первую
                                        );
    return entity;
}

/**
 * @brief Считаем центр глаза как центр масс описывающих его точек
 * @param points точки глаза
 */
cv::Point2d LocalGazeVectorTask::getEyeCenter(const std::vector<cv::Point2d>& points) const
{
    cv::Point2d centroid(0.0, 0.0);
    for (const auto& pt : points) {
        centroid.x += pt.x;
        centroid.y += pt.y;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();

    return centroid;
}

/// @brief Считаем радиус глаза
double LocalGazeVectorTask::getEyeRadius(
    const cv::Point2d& center, 
    const cv::Point2d& leftCorner) const
{
    auto delta = center - leftCorner;
    return cv::sqrt(delta.x * delta.x + delta.y * delta.y);
}


/// @brief Получаем вектор локального взгляда
cv::Vec3d LocalGazeVectorTask::getLocalGazeVector(const cv::Point2d& eyeCenter,
                                                    const cv::Point2d& irisCenter,
                                                    double eyeRadius) const
{
    cv::Vec3d endPoint = {irisCenter.x - eyeCenter.x, irisCenter.y - irisCenter.y, 0};
    double zSquared = eyeRadius * eyeRadius - irisCenter.x * irisCenter.x - irisCenter.y * irisCenter.y;
    ASSERT_PRINTF(zSquared >= 0, "Depth coord of local gaze vector negative!!!");
    endPoint[2] = cv::sqrt(zSquared);
    //отправляем нормализованный вектор
    return cv::normalize(endPoint);
}
