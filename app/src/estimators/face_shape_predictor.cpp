#include "face_shape_predictor.hpp"

// To opencv
#include "dlib/opencv.h"

#include "opencv2/opencv_modules.hpp"

#include "logging/logger.hpp"


const std::array<cv::Point3f, 6> FaceShapePredictor::kVirtualModelPoints = {
    cv::Point3f{0.0f, 0.0f, 0.0f},         // Нос 
    cv::Point3f{0.0f, -330.0f, -65.0f},    // Подбородок
    cv::Point3f{-225.0f, 170.0f, -135.0f}, // Левый угол глаза
    cv::Point3f{225.0f, 170.0f, -135.0f},  // Правый угол глаза
    cv::Point3f{-150.0f, -150.0f, -125.0f},// Левый угол рта
    cv::Point3f{150.0f, -150.0f, -125.0f}  // Правый угол рта
};


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
 * @brief Обновляем предиктор новым лицом
 * @param frame кадр изображения
 */
bool FaceShapePredictor::updateFace(const cv::Mat& frame)
{
    dlib::array2d<dlib::rgb_pixel> img;
    dlib::assign_image(img, dlib::cv_image<dlib::rgb_pixel>(frame));

    auto detectedImgsShapes = m_detector(img);
    //Сбрасываем текущее лицо
    m_curDetection.reset();
    if(detectedImgsShapes.size() == 1)
    {
        //Нашли одно лицо - возвращаем
        m_curDetection = m_sp(img, detectedImgsShapes.front());
    }else if(detectedImgsShapes.size() > 1)
    {
        //Если нашли больше, чем одно
        m_curDetection = m_sp(img, getPrimaryFaceRect(img, detectedImgsShapes));
    }
    //Не нашли ничего
    return m_curDetection.has_value();
}

/**
 * @brief Получаем выборку точек левого глаза
 * @return std::vector<cv::Point2i> подвыборка точек
 */
std::vector<cv::Point2i> FaceShapePredictor::getLeftEyePoints() const noexcept
{
    return getEyePoints(kLeftEyeStartPoint, kLeftEyeEndPoint);
}

/**
 * @brief Получаем выборку точек правого глаза
 * @return std::vector<cv::Point2i> подвыборка точек
 */
std::vector<cv::Point2i> FaceShapePredictor::getRightEyePoints() const noexcept
{
    return getEyePoints(kRightEyeStartPoint, kRightEyeEndPoint);
}

/**
 * @brief Получаем ограничивающий прямоугольник для левого глаза
 * @return cv::Rect ограничивающий прямоугольник
 */
cv::Rect FaceShapePredictor::getLeftEyeBoundingRect() const noexcept
{
    return getEyeBoundingRect(kLeftEyeStartPoint, kLeftEyeEndPoint);
}

/**
 * @brief Получаем ограничивающий прямоугольник для правого глаза
 * @return cv::Rect ограничивающий прямоугольник
 */
cv::Rect FaceShapePredictor::getRightEyeBoundingRect() const noexcept
{
    return getEyeBoundingRect(kRightEyeStartPoint, kRightEyeEndPoint);
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
 * @brief Получаем выборку точек глаз
 * @return std::vector<cv::Point2d> подвыборка точек
 */
std::vector<cv::Point2i> FaceShapePredictor::getEyePoints(std::size_t start, std::size_t end) const noexcept
{
    if(!m_curDetection)
        return {};
    std::vector<cv::Point2i> eyePoints;
    for (auto i = start; i <= end; ++i)
    {
        eyePoints.emplace_back(m_curDetection->part(i).x(), m_curDetection->part(i).y());
    }
    return eyePoints;
}

/**
 * @brief Получаем ограничивающий прямоугольник для глаза по точкам face_shape_landmark
 * @return cv::Rect ограничивающий прямоугольник
 */
cv::Rect FaceShapePredictor::getEyeBoundingRect(std::size_t start, std::size_t end) const noexcept
{
    if(!m_curDetection)
        return cv::Rect();
    return cv::boundingRect(getEyePoints(start, end));
}
