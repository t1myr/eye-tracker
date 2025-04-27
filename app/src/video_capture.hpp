#ifndef _VIDEO_CAPTURE_HPP_
#define _VIDEO_CAPTURE_HPP_

///video
#include "opencv2/videoio.hpp"

///base
#include "tasks/task.hpp"
#include "predictors/face_shape_predictor.hpp"

//calibrate
#include "camera_calibrator.hpp"

//gaze vector
#include "predictors/eye_gaze_tracker.hpp"

/// @brief Класс, осуществляющий захват видео кадра и его отрисовку
class VideoCapture : public Task
{
public:
    /// @brief Конструктор
    VideoCapture(const std::string& shapePredictorPath);

private:

    //---------------Работа с потоком-------------------------------------------
    /**
     * @brief Основная функция задачи
     */
    void mainFunc() override;

    /**
     * @brief Инициализация
     */
    void init() override;

    /**
     * @brief Принимаем сообщение от другой задачи
     * @param msg сообщение
     */
    void receiveMessage(Message &&msg) noexcept override {}

    //---------------Поиск лица-------------------------------------------------
    FaceShapePredictor m_facePredictor;

    //---------------Поиск лица-------------------------------------------------
    EyeGazeTracker m_gazeTracker{};

    //---------------Работа с отрисовкой кадра----------------------------------
    /**
     * @brief Рисуем маску лица
     * @param faceShape маска лица
     */
    void drawFaceMask(const dlib::full_object_detection& faceShape) const noexcept;
    /**
     * @brief Рисуем ограничивающие прямоугольники для глаз
     * @param faceShape маска лица
     */
    void drawEyeBoundingBox(const dlib::full_object_detection& faceShape) const noexcept;

    cv::Mat m_curFrame;

    //---------------Работа с камерой-------------------------------------------
    int m_deviceId; //Id камеры
    cv::VideoCaptureAPIs m_apiId; //Id используемого API
    cv::VideoCapture m_cap;
    std::unique_ptr<CameraCalibrator> m_calibrator; //Калибровка 
    std::vector<cv::Point3f> m_3dmodelPoints = {
        {0.0f, 0.0f, 0.0f},         // Нос (tip)
        {0.0f, -330.0f, -65.0f},    // Подбородок
        {-225.0f, 170.0f, -135.0f}, // Левый угол глаза
        {225.0f, 170.0f, -135.0f},  // Правый угол глаза
        {-150.0f, -150.0f, -125.0f},// Левый угол рта
        {150.0f, -150.0f, -125.0f}  // Правый угол рта
    };

    //---------------Работа с другими задачами----------------------------------
    ///empty now
};

#endif //_VIDEO_CAPTURE_HPP_

