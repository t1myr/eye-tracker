#ifndef _GLOBAL_GAZE_ESTIMATOR_HPP_
#define _GLOBAL_GAZE_ESTIMATOR_HPP_

//base
#include "dlib/image_processing.h"

//subtasks
#include "local_gaze_vector/local_gaze_vector_task.hpp"
#include "pose_estimation/head_pose_estimation_task.hpp"

//predictor
#include "face_shape_predictor.hpp"

/// @brief Вычислитель для глобального вектора взгляда
class GlobalGazeEstimator
{
public:
    struct GazeVector
    {
        cv::Vec3d startPoint;
        cv::Vec3d endPoint;
    };

    /// @brief Конструктор
    GlobalGazeEstimator(std::shared_ptr<CameraCalibrator> calib, const std::string& predictorPath);

    /// @brief Вычисляем вектор
    std::optional<GazeVector> estimate(const cv::Mat& frame);

private:
    LocalGazeVectorTask m_localGazeVectorTask;
    HeadPoseEstimationTask m_headPoseEstimationTask;
    FaceShapePredictor m_faceShapePredictor;

    struct
    {
        cv::Point2d left;
        cv::Point2d right;
    } m_leftCornerPoints;

    /// @brief Создаем конфиг для левого глаза
    LocalGazeVectorTask::EyeConfig makeLeftEyeConfig(const cv::Mat& frame);

    /// @brief Создаем конфиг для правого глаза
    LocalGazeVectorTask::EyeConfig makeRightEyeConfig(const cv::Mat& frame);
};

#endif