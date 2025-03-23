#ifndef _FACE_SHAPE_PREDICTOR_HPP_
#define _FACE_SHAPE_PREDICTOR_HPP_

#include "dlib/image_processing/frontal_face_detector.h"
#include "dlib/image_processing/shape_predictor.h"

/// Предиктор формы лица
class FaceShapePredictor
{
public:
    FaceShapePredictor(const std::string& filePath) noexcept;

    dlib::full_object_detection getFaceShape(const dlib::array2d<dlib::rgb_pixel>& img);

private:
    dlib::shape_predictor m_sp; //68 точек 
    dlib::frontal_face_detector m_detector;
};

#endif //_FACE_SHAPE_PREDICTOR_HPP_
