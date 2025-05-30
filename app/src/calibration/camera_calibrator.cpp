#include "camera_calibrator.hpp"

//cv
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"

//logging
#include "logging/logger.hpp"


/**
 * @brief Конструктор, сохраняющий путь к калибровочным данным
 * @param path 
 */
CameraCalibrator::CameraCalibrator(const std::shared_ptr<FrameCaptureEntity>& cap, const std::string& path) : 
                                                                    m_filePath(path),
                                                                    m_cap(cap)
{
    if (!load(m_filePath)) {
        spdlog::info("Calibration file not found or invalid. Starting calibration...");
        m_calibrated = calibrate();
        spdlog::info("Calibration... {}", m_calibrated ? "done" : "incomplete, errors occuried");
    } else {
        spdlog::info("Calibration parameters loaded from file: {}", path);
        m_calibrated = true;
    }
}

/**
 * @brief Конструктор по умолчанию
 */
CameraCalibrator::CameraCalibrator(const std::shared_ptr<FrameCaptureEntity>& cap) : 
                            CameraCalibrator(cap, CameraCalibrator::kCalibDataDefaultPath)
{

}

/**
 * @brief Калибровка
 */
bool CameraCalibrator::calibrate() 
{
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Mat> images;

    int captured = 0;
    while (captured < 15) {
        cv::Mat gray;
        cv::Mat frame = m_cap->nextFrame();

        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;

        bool found = cv::findChessboardCorners(gray, m_boardSize, corners, 
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cv::cornerSubPix(gray, corners, {11, 11}, {-1, -1},
                             {cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001});
            cv::drawChessboardCorners(frame, m_boardSize, corners, found);
            imagePoints.push_back(corners);
            images.push_back(frame.clone());
            ++captured;
            spdlog::info("Captured frame {}/15", captured);
        }

        cv::drawChessboardCorners(frame, m_boardSize, corners, found);
        cv::imshow("Calibration", frame);
        if (cv::waitKey(100) == 27) break;
    }
    cv::destroyAllWindows();

    if (imagePoints.size() < 5) {
        spdlog::warn("Not enough valid calibration images.");
        return false;
    }

    auto objectPoints = createObjectPoints(imagePoints.size());
    std::vector<cv::Mat> rvecs, tvecs;
    m_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    m_distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    double rms = cv::calibrateCamera(objectPoints, imagePoints, images[0].size(),
                                     m_cameraMatrix, m_distCoeffs, rvecs, tvecs);
    spdlog::info("Calibration RMS error: {}", rms);

    save(m_filePath);
    return true;
}

/// Проверка, выполнена ли калибровка
bool CameraCalibrator::isCalibrated() const
{
    return m_calibrated;
}
/**
 * @brief Получить матрицу камеры
 * @return const cv::Mat& 
 */
const cv::Mat& CameraCalibrator::getCameraMatrix() const {
    return m_cameraMatrix;
}
/**
 * @brief Получить коэффициенты дисторсии
 * @return const cv::Mat& 
 */
const cv::Mat& CameraCalibrator::getDistCoeffs() const {
    return m_distCoeffs;
}
/**
 * @brief Загружаем параметры из файла
 * @param filename 
 * @return true 
 * @return false 
 */
bool CameraCalibrator::load(const std::string& filename) noexcept
{
    cv::FileStorage fs(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_XML);
    if (!fs.isOpened()) 
    {
        spdlog::error("Cannot open file for calibration data");
        return false;
    }

    fs["camera_matrix"] >> m_cameraMatrix;
    fs["dist_coeffs"] >> m_distCoeffs;

    fs.release();
    return !m_cameraMatrix.empty() && !m_distCoeffs.empty();
}

/**
 * @brief Сохраняем калибровочные параметры 
 * @param filename 
 * @return true 
 * @return false 
 */
bool CameraCalibrator::save(const std::string& filename) const noexcept
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_XML);
    if (!fs.isOpened()) 
    {
        spdlog::error("Cannot open file for calibration data");
        return false;
    }

    fs << "camera_matrix" << m_cameraMatrix;
    fs << "dist_coeffs" << m_distCoeffs;

    fs.release();
    return true;
}

/**
 * @brief Генерация 3D-координат углов доски
 */
std::vector<std::vector<cv::Point3f>> CameraCalibrator::createObjectPoints(size_t numViews) {
    std::vector<std::vector<cv::Point3f>> objectPoints(numViews);
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < m_boardSize.height; ++i)
        for (int j = 0; j < m_boardSize.width; ++j)
            obj.emplace_back(j * m_squareSize, i * m_squareSize, 0.0f);
    std::fill(objectPoints.begin(), objectPoints.end(), obj);
    return objectPoints;
}
