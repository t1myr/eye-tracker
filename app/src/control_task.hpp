#ifndef _VIDEO_CAPTURE_HPP_
#define _VIDEO_CAPTURE_HPP_

///base
#include "tasks/task.hpp"
#include "predictors/face_shape_predictor.hpp"

///frame capturing
#include "frame_capture_entity/frame_capture_entity.hpp"

//calibrate
#include "calibration/camera_calibrator.hpp"

//gaze vector
#include "predictors/eye_gaze_tracker.hpp"


/// @brief Класс, осуществляющий захват видео кадра и его отрисовку
class ControlTask : public Task
{
public:
    /// @brief Конструктор
    ControlTask(const std::string& shapePredictorPath);

    /// @brief Задаем виртуальную сцену
    void setVirtualScene(Task* virtScene) { m_virtualScene = virtScene; }

private:

    //---------------Работа с потоком-------------------------------------------
    /**
     * @brief Инициализация
     */
    void init() override;

    /**
     * @brief Основная функция задачи
     */
    void mainFunc() override;

    //---------------Работа с сообщениями---------------------------------------
    /**
     * @brief Принимаем сообщение от другой задачи
     * @param msg сообщение
     */
    void receiveMessage(Message &&msg) override;

    //---------------Поиск лица-------------------------------------------------
    FaceShapePredictor m_facePredictor;

    //---------------Трекер взгляда---------------------------------------------
    std::unique_ptr<EyeGazeTracker> m_gazeTracker;

    //---------------Трекер взгляда---------------------------------------------
    Task* m_virtualScene{nullptr};

    //---------------Работа с отрисовкой кадра----------------------------------
    cv::Mat m_curFrame; //Текущий кадр

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

    /**
     * @brief Отрисовка текущего кадра
     * @param frameName имя кадра
     * @param frame кадр
     */
    void renderFrame(const std::string& frameName, const cv::Mat& frame);

    //---------------Работа с камерой-------------------------------------------
    std::shared_ptr<FrameCaptureEntity> m_cap;
    std::shared_ptr<CameraCalibrator> m_calibrator; //Калибровка 

    //---------------Работа с другими задачами----------------------------------
    Task* virtualScene{nullptr};
};

#endif //_VIDEO_CAPTURE_HPP_

