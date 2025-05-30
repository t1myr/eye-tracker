#include "eye_ellipse_fitter.hpp"

#include <opencv2/imgproc.hpp>


/// @brief Вычисляем эллипс по изображению
std::optional<cv::RotatedRect> EyeEllipseFitter::fit(const cv::Mat& eyeRoi)
{
    //Получаем бинарное изображение граней
    auto edges = getBinaryEdges(eyeRoi);
    //Находим все вертикальные контуры
    auto verticalContours = findVerticalContours(edges);
    //Получаем контуры зрачков
    auto irisContour = getIrisContour(verticalContours);

    //Если размер контура выше некоторого порога - запускаем алгоритм  
    if (irisContour.size() >= kIrisContourLengthThreshold)
        
        return cv::fitEllipse(irisContour);
    else
        return std::nullopt;
}

/// @brief Получаем бинарное изображение с гранями
cv::Mat EyeEllipseFitter::getBinaryEdges(const cv::Mat& eyeRoi)
{
    cv::Mat edges;
    cv::cvtColor(eyeRoi, edges, cv::COLOR_BGR2GRAY);
    cv::Canny(edges, edges, kCannyLowerBound, kCannyUpperBound);
    return edges;
}

/// @brief Получаем все вертикальные контуры на бинарном изображении
std::vector<std::vector<cv::Point>> EyeEllipseFitter::findVerticalContours(const cv::Mat& edges)
{
    std::vector<std::vector<cv::Point>> allContours;
    std::vector<std::vector<cv::Point>> verticalContours;
    //Ищем контуры
    cv::findContours(edges, allContours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    for (const auto& contour : allContours)
    {
        if (contour.size() < kIrisContourLengthThreshold) continue;

        cv::Vec4f line;
        cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);

        float dx = line[0];
        float dy = line[1];
        float angle = std::atan2(dy, dx) * 180.0f / CV_PI;

        if (std::fabs(std::fabs(angle) - 90.0f) <= 45.0f)
            verticalContours.emplace_back(contour);
    }
    return verticalContours;
}

/// @brief Получаем контур зрачка
std::vector<cv::Point> EyeEllipseFitter::getIrisContour(std::vector<std::vector<cv::Point>>& contours)
{
    //Если контуров меньше чем 2 - выходим
    if (contours.size() < 2)
        return {};

    //Сортируем контуры по длине и высоте
    std::sort(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            double lenA = cv::arcLength(a, false);
            double lenB = cv::arcLength(b, false);

            double heightA = std::fabs(a.front().y - a.back().y);
            double heightB = std::fabs(b.front().y - b.back().y);

            return (lenA > lenB) && (heightA > heightB);
        });

    //Сохраняем только первые два по длине
    std::vector<cv::Point> irisContour;
    irisContour.insert(irisContour.end(), contours[0].begin(), contours[0].end());
    irisContour.insert(irisContour.end(), contours[1].begin(), contours[1].end());

    return irisContour;
}
