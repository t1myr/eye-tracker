#ifndef _MAIN_TASK_HPP_
#define _MAIN_TASK_HPP_

///base
#include "tasks/task.hpp"

///frame capturing
#include "frame_capture_entity/frame_capture_entity.hpp"

//calibrate
#include "calibration/camera_calibrator.hpp"

//estimators
#include "estimators/global_gaze_estimator.hpp"


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

    //---------------Эстиматоры-------------------------------------------------
    std::unique_ptr<GlobalGazeEstimator> m_globalGazeEstimator;
    const std::string m_shapePredictorPath;

    //---------------Работа с задачами------------------------------------------
    Task* m_virtualScene{nullptr};

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
    cv::Mat m_curFrame; //Текущий кадр
};

#endif //_VIDEO_CAPTURE_HPP_

