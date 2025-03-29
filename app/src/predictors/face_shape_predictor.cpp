#include "face_shape_predictor.hpp"


//==================================================================================================
//---------------SHAPE PREDICTOR--------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Конструктор
 * @param путь к файлу с моделью shape_predictor с 68 точками
 */
FaceShapePredictor::FaceShapePredictor(const std::string& filePath) noexcept
{
    dlib::deserialize(filePath) >> m_sp;
    m_detector = dlib::get_frontal_face_detector();
}

//==================================================================================================
//---------------Обработка изображений--------------------------------------------------------------
//==================================================================================================
/**
 * @brief Получаем точки лица 
 * @param img картинка с лицом
 * @return dlib::full_object_detection 
 */
std::optional<dlib::full_object_detection> FaceShapePredictor::getFaceShape(
                                                        const dlib::array2d<dlib::rgb_pixel>& img)
{
    auto detectedImgsShapes = m_detector(img);
    if(detectedImgsShapes.size() == 1)
        //Нашли одно лицо - возвращаем
        return m_sp(img, detectedImgsShapes.front());
    else if(detectedImgsShapes.size() > 1)
        //Если нашли больше, чем одно
        return m_sp(img, getPrimaryFaceRect(img, detectedImgsShapes));
    //Не нашли ничего
    return {};
}

const dlib::rectangle& FaceShapePredictor::getPrimaryFaceRect(
                                        const dlib::array2d<dlib::rgb_pixel>& img, 
                                        const std::vector<dlib::rectangle>& faces) const noexcept
{
    assert(!faces.empty());
    return faces.front();
}
