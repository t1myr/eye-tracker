#ifndef _EYE_GAZE_TRACKER_HPP_
#define _EYE_GAZE_TRACKER_HPP_

//base
#include "opencv2/core.hpp"
#include "dlib/image_processing.h"

//calibration
#include "../calibration/camera_calibrator.hpp"


/// @brief Трекер глаза
class EyeGazeTracker 
{
public:
    /// @brief Конструктор
    EyeGazeTracker(std::shared_ptr<CameraCalibrator> calibrator);

    /// @brief Вычисляем позу лица
    bool estimateHeadPose(const std::vector<cv::Point2f>& refPoints);

    /// @brief Обновить трекер новым лицом и кадром
    void update(const dlib::full_object_detection& faceShape, const cv::Mat& frame);

    /// @brief Трекер готов к выдаче результатов
    bool ready() const { return m_hasPose && m_hasVector; }

    ///@brief  Получить глобальный вектор 
    cv::Vec3d getGlobalGazeVector() const;

    /// @brief Центры зрачков
    std::optional<cv::Point2f> getLastPupilCenterLeft() const;
    std::optional<cv::Point2f> getLastPupilCenterRight() const;

private:
    //---------------Калибровка-------------------------------------------------
    std::shared_ptr<CameraCalibrator> m_calibrator;

    //---------------Поза головы------------------------------------------------
    cv::Matx33d m_rotationMatrix;
    cv::Vec3d m_translationVector;
    bool m_hasPose{false};

    //---------------Вектор взгляда---------------------------------------------
    std::optional<cv::Vec3d> m_localGazeLeft;
    std::optional<cv::Vec3d> m_globalGazeLeft;
    std::optional<cv::Vec3d> m_localGazeRight;
    std::optional<cv::Vec3d> m_globalGazeRight;

    std::optional<cv::Point2f> m_pupilCenterLeft;
    std::optional<cv::Point2f> m_pupilCenterRight;

    bool m_hasVector{false};

    /// @brief Получаем ROI глаз
    std::pair<cv::Rect, cv::Rect> extractEyeROIs(const dlib::full_object_detection& shape, const cv::Mat& image);
    /// @brief Обновляем один глаз
    void updateSingleEye(
        const cv::Mat& frame,
        const cv::Rect& roi,
        std::optional<cv::Vec3d>& localGaze,
        std::optional<cv::Vec3d>& globalGaze,
        std::optional<cv::Point2f>& pupilCenter
    );
    /// @brief Обработка одного глаза
    std::optional<std::pair<cv::Vec3d, cv::Point2f>> processEye(const cv::Mat& eyeROI);
};

#endif //_EYE_GAZE_TRACKER_HPP_
