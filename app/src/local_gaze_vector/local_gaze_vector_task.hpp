#ifndef _LOCAL_GAZE_VECTOR_TASK_HPP_
#define _LOCAL_GAZE_VECTOR_TASK_HPP_

//base
#include <opencv2/core.hpp>
//tasks
#include "tasks/package_task.hpp"
#include "tasks/thread_pool.hpp"
//estimators
#include "estimators/eye_ellipse_fitter.hpp"


/// @brief Задача по вычислению локального вектора взгляда
class LocalGazeVectorTask : public PackageTask
{
public:
    struct EyeConfig
    {
        cv::Mat roi;
        std::vector<cv::Point2d> eyePoints;
    };

    /// @brief Тело запроса
    struct Body : PackageTask::ExecuteBody
    {
        EyeConfig left, right;
    };

    /// @brief Сущность для описания локального положения зрачка и взгляда
    struct EyeEntity
    {
        cv::Vec3d gazeVec;
        cv::Point2d eyeCenter;
    };

    /// @brief Тело результата
    struct Result : PackageTask::ExecuteBody
    {
        EyeEntity left, right;
    };

    /// @brief Конструктор
    LocalGazeVectorTask();
    
private:
    EyeEllipseFitter m_ellipseFitter {}; ///Вычисление эллипса
    ThreadPool* m_threadPool {nullptr};

    /// @brief Запускаем расчет задачи
    std::unique_ptr<ExecuteBody> execute(std::unique_ptr<ExecuteBody>&& body) override;
    /// @brief Вычисляем локальный вектор взгляда
    EyeEntity computeLocalGaze(const cv::Point2d& irisCenter, 
                                const std::vector<cv::Point2d>& points) const;

    /// @brief Считаем центр глаза
    cv::Point2d getEyeCenter(const std::vector<cv::Point2d>& points) const;
    /// @brief Считаем радиус глаза
    double getEyeRadius(const cv::Point2d& center, 
                        const cv::Point2d& leftCorner) const;
    /// @brief Получаем вектор локального взгляда
    cv::Vec3d getLocalGazeVector(const cv::Point2d& eyeCenter,
                                   const cv::Point2d& irisCenter,
                                   double eyeRadius) const;
};

#endif