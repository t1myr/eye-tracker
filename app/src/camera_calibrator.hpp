#ifndef _CAMERA_CALIBRATOR_HPP_
#define _CAMERA_CALIBRATOR_HPP_

///video
#include "opencv2/videoio.hpp"

/**
 * @brief Класс для калибровки камеры по шахматной доске
 */
class CameraCalibrator
{
public:
    inline static const std::string kCalibDataDefaultPath = "C:/Users/timur/calibrationData.xml";
    ///Конструктор по умолчанию
    CameraCalibrator(cv::VideoCapture& cap);
    ///Конструктор, сохраняющий путь к калибровочным данным
    CameraCalibrator(cv::VideoCapture& cap, const std::string& path);

    /// Выполнить калибровку камеры
    bool calibrate();
    /// Проверка, выполнена ли калибровка
    bool isCalibrated() const;

    /// Получить матрицу камеры
    const cv::Mat& getCameraMatrix() const;
    /// Получить коэффициенты дисторсии
    const cv::Mat& getDistCoeffs() const;
    /// Загружаем параметры
    bool load(const std::string& path) noexcept;
    /// Сохраняем калибровочные параметры 
    bool save(const std::string& path) const noexcept;

private:
    //Результаты калибровки
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;
    bool m_calibrated = false;
    //Video capture
    cv::VideoCapture& m_cap;
    std::string m_filePath;

    // Настройки шахматной доски
    cv::Size m_boardSize = {7, 7};
    float m_squareSize = 2.6f / 100.0f; // метры

    // Хранение точек
    std::vector<std::vector<cv::Point3f>> m_objectPoints;
    std::vector<std::vector<cv::Point2f>> m_imagePoints;

    /// Генерация 3D-координат углов доски
    std::vector<std::vector<cv::Point3f>> createObjectPoints(size_t numViews);
};

#endif