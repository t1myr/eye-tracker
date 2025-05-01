#ifndef _FACE_SHAPE_PREDICTOR_HPP_
#define _FACE_SHAPE_PREDICTOR_HPP_

#include <optional>

//predictors
#include "dlib/image_processing/frontal_face_detector.h"
#include "dlib/image_processing/shape_predictor.h"

//opencv
#include "opencv2/imgproc.hpp"


/// @brief Предиктор формы лица
class FaceShapePredictor
{
public:

    //---------------Константы--------------------------------------------------
    static constexpr std::size_t kLeftEyeStartPoint = 36;
    static constexpr std::size_t kLeftEyeEndPoint = 41;
    static constexpr std::size_t kRightEyeStartPoint = 42;
    static constexpr std::size_t kRightEyeEndPoint = 47;
    static const std::array<cv::Point3f, 6> kVirtualModelPoints;

    /// @brief Конструктор
    FaceShapePredictor(const std::string& filePath) noexcept;

    //---------------Определение объектов на изображении------------------------
    /**
     * @brief Получаем точки лица
     * @param frame кадр изображения
     */
    std::optional<dlib::full_object_detection> getFaceShape(const cv::Mat& frame);
    /**
     * @brief Получаем ограничивающий прямоугольник для левого глаза
     * @param shape точки лица
     * @return cv::Rect ограничивающий прямоугольник
     */
    cv::Rect getLeftEyeBoundingRect(const dlib::full_object_detection& shape) const noexcept;
    /**
     * @brief Получаем ограничивающий прямоугольник для правого глаза
     * @param shape точки лица
     * @return cv::Rect ограничивающий прямоугольник
     */
    cv::Rect getRightEyeBoundingRect(const dlib::full_object_detection& shape) const noexcept;

    /**
     * @brief Получаем референсные точки для лица
     * @return std::vector<cv::Point2f> 
     */
    std::vector<cv::Point2f> getRefFacePoints() const;
private:
    /**
     * @brief Получаем главное лицо на картинке
     * @param img изображение
     * @param faces прямоугольники, ограничивающие лица
     * @return const dlib::rectangle& 
     */
    const dlib::rectangle& getPrimaryFaceRect(
                                        const dlib::array2d<dlib::rgb_pixel>& img, 
                                        const std::vector<dlib::rectangle>& faces) const noexcept;
    /**
     * @brief Получаем ограничивающий прямоугольник для глаза по точкам face_shape_landmark
     * @param shape точки лица
     * @return cv::Rect ограничивающий прямоугольник
     */
    cv::Rect getEyeBoundingRect(const dlib::full_object_detection& shape, 
                                                std::size_t start, std::size_t end) const noexcept;


    dlib::shape_predictor m_sp; //68 точек 
    dlib::frontal_face_detector m_detector; //детектор лица
    std::optional<dlib::full_object_detection> m_curDetection;
};

#endif //_FACE_SHAPE_PREDICTOR_HPP_
