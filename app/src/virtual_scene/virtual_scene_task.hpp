#ifndef _VIRTUAL_SCENE_HPP_
#define _VIRTUAL_SCENE_HPP_

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include "tasks/task.hpp"


/// @brief Класс, описывающий виртуальную сцену
class VirtualScene : public Task
{
public:
    struct SceneData {
        std::vector<cv::Point3d> cubeVertices;   // Вершины параллелепипеда
        std::optional<cv::Point3d> lastIntersectionPoint;       // Последняя точка пересечения
        cv::Vec3d gazeVector;                    // Последний вектор взгляда
    };

    /// @brief Конструктор
    VirtualScene();

    /// @brief Задаем задачу управления
    void setCtrl(Task* ctrl) { m_ctrl = ctrl; }

    /// @brief Обработка новой точки
    void handleNewPoint(const cv::Vec3d& gazePoint);

    /// @brief Готова ли к отрисовке сцена? 
    bool ready() const { return m_ready; }

    /// @brief Получаем все данные сцены для отрисовки
    SceneData getSceneData() const;

private:
    //---------------Работа с потоком-------------------------------------------
    /**
     * @brief Инициализация в потоке
     */
    void init() override;

    /**
     * @brief Основная функция задачи
     */
    void mainFunc() override;

    //---------------Работа с сообщениями---------------------------------------
    Task* m_ctrl{ nullptr };

    /**
     * @brief Принимаем сообщение от другой задачи
     * @param msg сообщение
     */
    void receiveMessage(Message &&msg) override;

    static constexpr uint8_t kPointsForSurface = 8;
    bool m_ready{false}; //Готова ли сцена к отрисовке
    std::vector<cv::Vec3d> m_points; //Точки виртуальной плоскости

    cv::Vec3d m_planeNormal;
    double m_planeD;

    std::vector<cv::Point3d> m_cubeVertices; // 8 вершин параллелепипеда
    std::optional<cv::Point3d> m_lastIntersection;
    cv::Vec3d m_lastGaze;

    void buildSurface();
    std::optional<cv::Point3d> intersectRayWithPlane(const cv::Vec3d& ray) const;

    /// @brief Генерируем кадр виртуальной сцены
    cv::Mat getSceneFrame(const VirtualScene::SceneData& sceneData);
};

#endif //_VIRTUAL_SCENE_HPP_