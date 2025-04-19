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
//---------------Определение объектов на изображении------------------------------------------------
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
    //Сбрасываем текущее лицо
    m_curDetection.reset();
    if(detectedImgsShapes.size() == 1)
        //Нашли одно лицо - возвращаем
        m_curDetection = m_sp(img, detectedImgsShapes.front());
    else if(detectedImgsShapes.size() > 1)
        //Если нашли больше, чем одно
        m_curDetection = m_sp(img, getPrimaryFaceRect(img, detectedImgsShapes));
    //Не нашли ничего
    return m_curDetection;
}

/**
 * @brief Получаем ограничивающий прямоугольник для левого глаза
 * @param shape точки лица
 * @return cv::Rect ограничивающий прямоугольник
 */
cv::Rect FaceShapePredictor::getLeftEyeBoundingRect(const dlib::full_object_detection& shape) const noexcept
{
    return getEyeBoundingRect(shape, kLeftEyeStartPoint, kLeftEyeEndPoint);
}

/**
 * @brief Получаем ограничивающий прямоугольник для правого глаза
 * @param shape точки лица
 * @return cv::Rect ограничивающий прямоугольник
 */
cv::Rect FaceShapePredictor::getRightEyeBoundingRect(const dlib::full_object_detection& shape) const noexcept
{
    return getEyeBoundingRect(shape, kRightEyeStartPoint, kRightEyeEndPoint);
}

/**
 * @brief Получаем референсные точки для лица
 * @return std::vector<cv::Point2f> 
 */
std::vector<cv::Point2f> FaceShapePredictor::getRefFacePoints() const
{
    if(!m_curDetection.has_value())
        throw std::logic_error("Currrent detection must be valid");

    return {
        cv::Point2f(m_curDetection->part(30).x(), m_curDetection->part(30).y()), // Нос
        cv::Point2f(m_curDetection->part(8).x(),  m_curDetection->part(8).y()),  // Подбородок
        cv::Point2f(m_curDetection->part(36).x(), m_curDetection->part(36).y()), // Левый глаз
        cv::Point2f(m_curDetection->part(45).x(), m_curDetection->part(45).y()), // Правый глаз
        cv::Point2f(m_curDetection->part(48).x(), m_curDetection->part(48).y()), // Левая часть рта
        cv::Point2f(m_curDetection->part(54).x(), m_curDetection->part(54).y())  // Правая часть рта
    };
}

/**
 * @brief Получаем главное лицо на картинке
 * @param img изображение
 * @param faces прямоугольники, ограничивающие лица
 * @return const dlib::rectangle& 
 */
const dlib::rectangle& FaceShapePredictor::getPrimaryFaceRect(
                                        const dlib::array2d<dlib::rgb_pixel>& img, 
                                        const std::vector<dlib::rectangle>& faces) const noexcept
{
    assert(!faces.empty());
    return faces.front();
}

/**
 * @brief Получаем ограничивающий прямоугольник для глаза по точкам face_shape_landmark
 * @param shape точки лица
 * @return cv::Rect ограничивающий прямоугольник
 */
cv::Rect FaceShapePredictor::getEyeBoundingRect(const dlib::full_object_detection& shape, 
                                                std::size_t start, std::size_t end) const noexcept
{
    std::vector<cv::Point> eyePoints;
    for (auto i = start; i <= end; ++i)
    {
        eyePoints.emplace_back(shape.part(i).x(), shape.part(i).y());
    }
    return cv::boundingRect(eyePoints);
}
