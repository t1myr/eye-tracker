#ifndef _EYE_GAZE_TRACKER_HPP_
#define _EYE_GAZE_TRACKER_HPP_

#include "opencv2/core.hpp"
#include "dlib/image_processing.h"

/// @brief Трекер глаза
class EyeGazeTracker 
{
public:
    /// @brief Конструктор
    EyeGazeTracker();

    /// @brief Установить матрицу вращения и вектор трансляции от Head Pose Estimation
    void setHeadPose(const cv::Matx33d& rotation, const cv::Vec3d& translation);

    /// @brief Обновить трекер новым лицом и кадром
    void update(const dlib::full_object_detection& faceShape, const cv::Mat& frame);

    ///@brief  Получить глобальный вектор 
    std::optional<cv::Vec3d> getGlobalGazeVector() const;

    /// @brief Центры зрачков
    std::optional<cv::Point2f> getLastPupilCenterLeft() const;
    std::optional<cv::Point2f> getLastPupilCenterRight() const;

private:
    cv::Matx33d rotationMatrix_;
    cv::Vec3d translationVector_;
    bool hasPose_ = false;

    std::optional<cv::Vec3d> m_localGazeLeft;
    std::optional<cv::Vec3d> m_globalGazeLeft;
    std::optional<cv::Vec3d> m_localGazeRight;
    std::optional<cv::Vec3d> m_globalGazeRight;

    std::optional<cv::Point2f> m_pupilCenterLeft;
    std::optional<cv::Point2f> m_pupilCenterRight;

    void updateSingleEye(
        const cv::Mat& frame,
        const cv::Rect& roi,
        std::optional<cv::Vec3d>& localGaze,
        std::optional<cv::Vec3d>& globalGaze,
        std::optional<cv::Point2f>& pupilCenter
    );
    /// @brief Получаем из формы регионы с глазами
    std::pair<cv::Rect, cv::Rect> extractEyeROIs(const dlib::full_object_detection& shape, const cv::Mat& image);
    /// @brief Обработка одного глаза
    std::optional<std::pair<cv::Vec3d, cv::Point2f>> processEye(const cv::Mat& eyeROI);
};

#endif //_EYE_GAZE_TRACKER_HPP_
