#include "face_shape_predictor.hpp"

/**===================================================================================
 * @brief Конструктор
 * @param путь к файлу с моделью shape_predictor с 68 точками
 */
FaceShapePredictor::FaceShapePredictor(const std::string& filePath) noexcept
{
    dlib::deserialize(filePath) >> m_sp;
    m_detector = dlib::get_frontal_face_detector();
}

/**
 * @brief Получаем точки лица 
 * @param img картинка с лицом
 * @return dlib::full_object_detection 
 */
dlib::full_object_detection FaceShapePredictor::getFaceShape(const dlib::array2d<dlib::rgb_pixel>& img)
{
    auto detectedImgsShapes = m_detector(img);
    if(detectedImgsShapes.size() == 1)
    {
        //нашли лицо - возвращаем
        return m_sp(img, detectedImgsShapes.front());
    }else if(detectedImgsShapes.size() == 0)
    {
        //Если не нашли - что вернуть?
    }else
    {
        //Если нашли больше, чем одно
    }
}
