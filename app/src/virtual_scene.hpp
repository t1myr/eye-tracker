#ifndef _VIRTUAL_SCENE_HPP_
#define _VIRTUAL_SCENE_HPP_

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"


/// @brief Класс, описывающий виртуальную сцену
class VirtualScene
{
public:
    VirtualScene() = default;

    /// @brief Обработка новой точки
    void handleNewPoint(const cv::Vec3d& gazePoint);

    /// @brief Готова ли к отрисовке сцена? 
    bool ready() const;

    /// @brief Получаем точки сцены для отрисовки
    // ??? getScenePoints() const noexcept;

private:
    static constexpr uint8_t kPointsForSurface = 8;
    bool m_ready{false}; //Готова ли сцена к отрисовке
    std::vector<cv::Vec3d> m_points; //Точки виртуальной плоскости
};

#endif //_VIRTUAL_SCENE_HPP_