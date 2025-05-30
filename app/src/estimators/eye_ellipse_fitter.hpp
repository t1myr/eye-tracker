#ifndef _EYE_ELLIPSE_FITTER_HPP_
#define _EYE_ELLIPSE_FITTER_HPP_

#include <opencv2/core.hpp>


/// @brief Класс для поиска эллипса в заданной области
class EyeEllipseFitter 
{
public:
    /// @brief Вычисляем эллипс по изображению
    std::optional<cv::RotatedRect> fit(const cv::Mat& eyeRoi);
private:

    /// @brief Константы
    static constexpr uint32_t kCannyLowerBound = 100;
    static constexpr uint32_t kCannyUpperBound = 200;

    static constexpr uint32_t kIrisContourLengthThreshold = 5;

    /// @brief Получаем бинарное изображение с гранями
    cv::Mat getBinaryEdges(const cv::Mat& eyeRoi);

    /// @brief Получаем все вертикальные контуры на изображении
    std::vector<std::vector<cv::Point>> findVerticalContours(const cv::Mat& edges);

    /// @brief Получаем контур зрачка
    std::vector<cv::Point> getIrisContour(std::vector<std::vector<cv::Point>>& contours);
};



#endif // _EYE_ELLIPSE_FITTER_HPP_