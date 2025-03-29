#ifndef _FACE_SHAPE_PREDICTOR_HPP_
#define _FACE_SHAPE_PREDICTOR_HPP_

#include <optional>

//predictors
#include "dlib/image_processing/frontal_face_detector.h"
#include "dlib/image_processing/shape_predictor.h"

/// @brief Предиктор формы лица
class FaceShapePredictor
{
public:
    /// @brief Конструктор
    FaceShapePredictor(const std::string& filePath) noexcept;

    //---------------Обработка изображений--------------------------------------
   std::optional<dlib::full_object_detection> getFaceShape(const dlib::array2d<dlib::rgb_pixel>& img);

private:
    const dlib::rectangle& getPrimaryFaceRect(
                                        const dlib::array2d<dlib::rgb_pixel>& img, 
                                        const std::vector<dlib::rectangle>& faces) const noexcept;


    dlib::shape_predictor m_sp; //68 точек 
    dlib::frontal_face_detector m_detector;
};

#endif //_FACE_SHAPE_PREDICTOR_HPP_
