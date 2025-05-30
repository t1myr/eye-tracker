#ifndef _HEAD_POSE_ESTIMATION_TASK_HPP_
#define _HEAD_POSE_ESTIMATION_TASK_HPP_

#include "tasks/package_task.hpp"
#include "calibration/camera_calibrator.hpp"


/// @brief Задача по вычислению позы головы
class HeadPoseEstimationTask : public PackageTask
{
public:
    /// @brief Тело запроса
    struct Body : PackageTask::ExecuteBody
    {
        std::vector<cv::Point2f> refPoints;
        Body() = default;
        Body(const std::vector<cv::Point2f>& points) : refPoints(points) {}
    };

    /// @brief Тело результата
    struct Result : PackageTask::ExecuteBody
    {
        cv::Matx33d rotation; //Матрица поворота головы
        cv::Vec3d translation; //Вектор перемещения
    };

    /// @brief Конструктор
    HeadPoseEstimationTask(std::shared_ptr<CameraCalibrator> calibrator);
    
private:
    /// @brief Запускаем расчет задачи
    std::unique_ptr<ExecuteBody> execute(std::unique_ptr<ExecuteBody>&& body) override;

    ///Калибровка
    std::shared_ptr<CameraCalibrator> m_calibrator;
};

#endif