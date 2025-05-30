#include "head_pose_estimation_task.hpp"

#include <opencv2/calib3d.hpp>

#include "utils/assert.hpp"
#include "utils/cast.hpp"
#include "estimators/face_shape_predictor.hpp"

//==================================================================================================
//---------------HEAD POSE ESTIMATION TASK----------------------------------------------------------
//==================================================================================================
/// @brief Конструктор
HeadPoseEstimationTask::HeadPoseEstimationTask(std::shared_ptr<CameraCalibrator> calibrator) :
    PackageTask("pose", MessageDispatcher::get()),
    m_calibrator(calibrator)
{

}

/// @brief Запускаем расчет задачи
std::unique_ptr<PackageTask::ExecuteBody> HeadPoseEstimationTask::execute(std::unique_ptr<ExecuteBody>&& body)
{
    //Получаем переданные точки
    auto points = dynamicCastPtr(Body, body.get());

    auto result = std::make_unique<Result>();

    cv::Vec3d rvec;

    //Используем solvePnP для разрешения позы головы
    auto res = cv::solvePnP(
        FaceShapePredictor::kVirtualModelPoints, points->refPoints,
        m_calibrator->getCameraMatrix(), m_calibrator->getDistCoeffs(),
        rvec, result->translation
    );

    if(res)
    {   
        //Получилось посчитать - отправляем результат
        cv::Rodrigues(rvec, result->rotation);
        return result;
    }else
    {
        //Иначе результата нет
        return nullptr;
    }
}
