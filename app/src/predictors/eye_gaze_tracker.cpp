#include "eye_gaze_tracker.hpp"

//base
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

//predictor
#include "face_shape_predictor.hpp"

/**
 * @brief Конструктор
 * @param calibrator калибровка
 */
EyeGazeTracker::EyeGazeTracker(std::shared_ptr<CameraCalibrator> calibrator) : m_calibrator(calibrator)
{
    if(!m_calibrator)
        throw std::logic_error("Empty calibrator!!!");
}

/**
 * @brief Вычисляем позу лица
 * @param refPoints Референсные плоские точки лица
 * @return bool результат расчета
 */
bool EyeGazeTracker::estimateHeadPose(const std::vector<cv::Point2f>& refPoints) 
{
    cv::Vec3d rvec;

    m_hasPose = cv::solvePnP(
        FaceShapePredictor::kVirtualModelPoints, refPoints,
        m_calibrator->getCameraMatrix(), m_calibrator->getDistCoeffs(),
        rvec, m_translationVector
    );

    if(m_hasPose)
        cv::Rodrigues(rvec, m_rotationMatrix);

    return m_hasPose;
}

/**
 * @brief Обновить трекер новым кадром и маской лица
 * @param shape область лица
 * @param frame кадр с камеры
 */
void EyeGazeTracker::update(const dlib::full_object_detection& shape, const cv::Mat& frame) 
{
    /// Если нет валидной позы головы, ничего не делаем
    if(!m_hasPose)
        return;

    // Вычисляес ROI
    auto [leftROI, rightROI] = extractEyeROIs(shape, frame);

    // Обновляем левый глаз
    updateSingleEye(frame, leftROI, m_localGazeLeft, m_globalGazeLeft, m_pupilCenterLeft);
    // Обновляем правый глаз
    updateSingleEye(frame, rightROI, m_localGazeRight, m_globalGazeRight, m_pupilCenterRight);
}

/**
 * @brief Получаем глобальный вектор взгляда
 * @return cv::Vec3d
 */
cv::Vec3d EyeGazeTracker::getGlobalGazeVector() const 
{
    if(!m_hasPose)
        throw std::logic_error("Empty pose value!!!");
    if (m_globalGazeLeft && m_globalGazeRight) {
        cv::Vec3d avg = cv::normalize((m_globalGazeLeft.value() + m_globalGazeRight.value()) * 0.5);
        return avg;
    } else if (m_globalGazeLeft) {
        return *m_globalGazeLeft;
    } else if (m_globalGazeRight) {
        return *m_globalGazeRight;
    } else {
        throw std::logic_error("Invalid gaze vector!!!");
    }
}

/**
 * @brief Центр левого зрачка
 * @return std::optional<cv::Point2f> 
 */
std::optional<cv::Point2f> EyeGazeTracker::getLastPupilCenterLeft() const 
{
    return m_pupilCenterLeft;
}

/**
 * @brief Центр правого зрачка
 * @return std::optional<cv::Point2f> 
 */
std::optional<cv::Point2f> EyeGazeTracker::getLastPupilCenterRight() const {
    return m_pupilCenterRight;
}

/**
 * @brief Получаем ROI глаз 
 * @param shape 
 * @param image 
 * @return std::pair<cv::Rect, cv::Rect> 
 */
std::pair<cv::Rect, cv::Rect> EyeGazeTracker::extractEyeROIs(const dlib::full_object_detection& shape, const cv::Mat& image) {
    auto getBoundingBox = [&](int start, int end) {
        int minX = INT_MAX, minY = INT_MAX, maxX = 0, maxY = 0;
        for (int i = start; i <= end; ++i) {
            const auto& p = shape.part(i);
            minX = std::min(minX, static_cast<int>(p.x()));
            minY = std::min(minY, static_cast<int>(p.y()));
            maxX = std::max(maxX, static_cast<int>(p.x()));
            maxY = std::max(maxY, static_cast<int>(p.y()));
        }
        int pad = 5;
        cv::Rect roi(minX - pad, minY - pad, (maxX - minX) + 2 * pad, (maxY - minY) + 2 * pad);
        roi &= cv::Rect(0, 0, image.cols, image.rows);
        return roi;
    };

    return {
        getBoundingBox(FaceShapePredictor::kLeftEyeStartPoint, FaceShapePredictor::kLeftEyeEndPoint),
        getBoundingBox(FaceShapePredictor::kRightEyeStartPoint, FaceShapePredictor::kRightEyeEndPoint)
    };
}

/**
 * @brief Обновление одного глаза
 * @param frame кадр с камеры
 * @param roi регион с глазом
 * @param localGaze локальный вектор взгляда
 * @param globalGaze глобальный вектор взгляда
 * @param pupilCenter центр зрачка
 */
void EyeGazeTracker::updateSingleEye(
    const cv::Mat& frame,
    const cv::Rect& roi,
    std::optional<cv::Vec3d>& localGaze,
    std::optional<cv::Vec3d>& globalGaze,
    std::optional<cv::Point2f>& pupilCenter
) 
{
    cv::Mat eyeImg = frame(roi);
    auto result = processEye(eyeImg);

    if (result.has_value()) {
        localGaze = result->first;
        pupilCenter = result->second + cv::Point2f(roi.x, roi.y);
        if (m_hasPose) {
            globalGaze = cv::normalize(m_rotationMatrix * localGaze.value());
        }
    } else {
        localGaze = std::nullopt;
        globalGaze = std::nullopt;
        pupilCenter = std::nullopt;
    }
}

std::optional<std::pair<cv::Vec3d, cv::Point2f>> EyeGazeTracker::processEye(const cv::Mat& eyeROI) 
{
    cv::Mat gray, blurred, edges;
    cv::cvtColor(eyeROI, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::Canny(blurred, edges, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double bestScore = 1e9;
    cv::RotatedRect bestEllipse;

    for (const auto& c : contours) {
        if (c.size() < 5) continue;
        auto ellipse = cv::fitEllipse(c);
        double major = std::max(ellipse.size.width, ellipse.size.height);
        double minor = std::min(ellipse.size.width, ellipse.size.height);
        double ecc = std::sqrt(1.0 - (minor * minor) / (major * major));
        if (ecc < 0.8) {
            double dx = ellipse.center.x - eyeROI.cols / 2.0;
            double dy = ellipse.center.y - eyeROI.rows / 2.0;
            double score = std::hypot(dx, dy);
            if (score < bestScore) {
                bestScore = score;
                bestEllipse = ellipse;
            }
        }
    }

    if (bestScore >= 1e9) return std::nullopt;

    double dx = (bestEllipse.center.x - eyeROI.cols / 2.0) / (eyeROI.cols / 2.0);
    double dy = (bestEllipse.center.y - eyeROI.rows / 2.0) / (eyeROI.rows / 2.0);
    cv::Vec3d localGaze = cv::normalize(cv::Vec3d(dx, dy, 1.0));
    return std::make_pair(localGaze, bestEllipse.center);
}
