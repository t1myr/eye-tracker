#ifndef _SFM_HPP_
#define _SFM_HPP_

#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <vector>
#include <iostream>

// Структура для функции стоимости Ceres
struct ReprojectionError 
{
    cv::Point2f observed_pt; // Наблюдаемая 2D точка
    cv::Mat K;                // Калибровочная матрица

    ReprojectionError(const cv::Point2f& pt, const cv::Mat& K)
        : observed_pt(pt), K(K) {}

    template <typename T>
    bool operator()(const T* const camera, 
                    const T* const point,
                    T* residuals) const {
        // Параметры камеры: [angle_axis (3), translation (3)]
        const T* rotation = camera;
        const T* translation = camera + 3;

        // Преобразование 3D точки в систему координат камеры
        T pt[3];
        pt[0] = T(point[0]);
        pt[1] = T(point[1]);
        pt[2] = T(point[2]);
        
        T camera_pt[3];
        ceres::AngleAxisRotatePoint(rotation, pt, camera_pt);
        
        camera_pt[0] += translation[0];
        camera_pt[1] += translation[1];
        camera_pt[2] += translation[2];

        // Проецирование на изображение
        const T fx = T(K.at<double>(0,0));
        const T fy = T(K.at<double>(1,1));
        const T cx = T(K.at<double>(0,2));
        const T cy = T(K.at<double>(1,2));

        T xp = camera_pt[0] / camera_pt[2];
        T yp = camera_pt[1] / camera_pt[2];
        
        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;

        // Ошибка репроекции
        residuals[0] = predicted_x - T(observed_pt.x);
        residuals[1] = predicted_y - T(observed_pt.y);

        return true;
    }
};


class SFM 
{

public:
    /// @brief Конструктор
    SFM(const cv::Mat& camera_matrix, const std::vector<std::vector<cv::Point2f>>& points);

    // Основной метод запуска обработки
    void run();

    /// @brief Выдача результата
    inline const std::vector<cv::Point3f>& get3DPoints() const { return world_points; }

private:
    cv::Mat m_cameraMatrix;                                      // Калибровочная матрица камеры
    std::vector<cv::Mat> Rvecs, tvecs;              // Позы камер (всех кадров)
    std::vector<cv::Point3f> world_points;          // 3D точки (общие для всех кадров)
    std::vector<std::vector<cv::Point2f>> all_pts;  // Все 2D точки (по кадрам)
    bool initialized = false;                       // Флаг инициализации

    // Инициализация по первым двум кадрам
    void initializeFirstTwoFrames();

    // Триангуляция точек между двумя кадрами
    void triangulatePoints(int idx1, int idx2);

    // Добавление нового кадра
    void addNextFrame(size_t frame_idx);

    // Простейший Bundle Adjustment
    void bundleAdjustment();

    // Фиксация масштаба
    void fixScale();
};

#endif // _SFM_HPP_