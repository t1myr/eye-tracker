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
     * @brief Обновляем предиктор новым лицом
     * @param frame кадр изображения
     */
    bool updateFace(const cv::Mat& frame);

    /**
     * @brief Получаем точки лица
     */
    std::optional<dlib::full_object_detection> getFaceShape() const
    { return m_curDetection; }

    /**
     * @brief Получаем выборку точек левого глаза
     * @return std::vector<cv::Point2d> подвыборка точек
     */
    std::vector<cv::Point2i> getLeftEyePoints() const noexcept;

    /**
     * @brief Получаем выборку точек правого глаза
     * @return std::vector<cv::Point2d> подвыборка точек
     */
    std::vector<cv::Point2i> getRightEyePoints() const noexcept;

    /**
     * @brief Получаем ограничивающий прямоугольник для левого глаза
     * @return cv::Rect ограничивающий прямоугольник
     */
    cv::Rect getLeftEyeBoundingRect() const noexcept;
    /**
     * @brief Получаем ограничивающий прямоугольник для правого глаза
     * @return cv::Rect ограничивающий прямоугольник
     */
    cv::Rect getRightEyeBoundingRect() const noexcept;

    /**
     * @brief Получаем референсные точки для лица
     * @return std::vector<cv::Point2f> 
     */
    std::vector<cv::Point2f> getRefFacePoints() const;
private:
    dlib::shape_predictor m_sp; //68 точек 
    dlib::frontal_face_detector m_detector; //детектор лица
    std::optional<dlib::full_object_detection> m_curDetection;

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
     * @brief Получаем выборку точек глаз
     * @return std::vector<cv::Point2d> подвыборка точек
     */
    std::vector<cv::Point2i> getEyePoints(std::size_t start, std::size_t end) const noexcept;

    /**
     * @brief Получаем ограничивающий прямоугольник для глаза по точкам face_shape_landmark
     * @return cv::Rect ограничивающий прямоугольник
     */
    cv::Rect getEyeBoundingRect(std::size_t start, std::size_t end) const noexcept;
};

#endif //_FACE_SHAPE_PREDICTOR_HPP_
