#include "global_gaze_estimator.hpp"

#include "utils/cast.hpp"


/// @brief Конструктор
GlobalGazeEstimator::GlobalGazeEstimator(std::shared_ptr<CameraCalibrator> calib, 
                                        const std::string& predictorPath) :
    m_localGazeVectorTask(),
    m_headPoseEstimationTask(calib),
    m_faceShapePredictor(predictorPath)
{
    m_localGazeVectorTask.start();
    m_headPoseEstimationTask.start();
}

/// @brief Вычисляем вектор
std::optional<GlobalGazeEstimator::GazeVector> GlobalGazeEstimator::estimate(const cv::Mat& frame)
{
    //Обновляем предиктор
    if(!m_faceShapePredictor.updateFace(frame))
        return std::nullopt;

    auto lgvBody = std::make_unique<LocalGazeVectorTask::Body>();
    auto hpeBody = std::make_unique<HeadPoseEstimationTask::Body>();
    lgvBody->left = makeLeftEyeConfig(frame);
    lgvBody->right = makeRightEyeConfig(frame);

    hpeBody->refPoints = m_faceShapePredictor.getRefFacePoints();

    auto localGazeFuture = m_localGazeVectorTask.post(std::move(lgvBody));
    auto headPoseFuture = m_headPoseEstimationTask.post(std::move(hpeBody));

    auto localGazePtr = localGazeFuture.get();
    auto headPosePtr = headPoseFuture.get();

    //Если что то не получилось посчитать - выходим
    if(!localGazePtr || !headPosePtr)
        return std::nullopt;
    //Кастуем результаты
    auto localGaze = *dynamicCastPtr(LocalGazeVectorTask::Result, localGazePtr.get());
    auto headPose = *dynamicCastPtr(HeadPoseEstimationTask::Result, headPosePtr.get());

    //1. Посчитать трехмерную позицию начала вектора взгляда
    //2. Перевести этот вектор в систему координат головы
    //3. Перевести вектор в систему координат камеры
    //4. Вернуть результат

    return std::nullopt;
}


/// @brief Создаем конфиг для левого глаза
LocalGazeVectorTask::EyeConfig GlobalGazeEstimator::makeLeftEyeConfig(const cv::Mat& frame)
{
    auto rect = m_faceShapePredictor.getLeftEyeBoundingRect();
    m_leftCornerPoints.left = cv::Point2d(rect.x, rect.y);
    LocalGazeVectorTask::EyeConfig cfg;
    cfg.roi = frame(rect);
    cfg.eyePoints = m_faceShapePredictor.getLeftEyePoints();
    return cfg;
}

/// @brief Создаем конфиг для правого глаза
LocalGazeVectorTask::EyeConfig GlobalGazeEstimator::makeRightEyeConfig(const cv::Mat& frame)
{
    auto rect = m_faceShapePredictor.getRightEyeBoundingRect();
    m_leftCornerPoints.right = cv::Point2d(rect.x, rect.y);
    LocalGazeVectorTask::EyeConfig cfg;
    cfg.roi = frame(rect);
    cfg.eyePoints = m_faceShapePredictor.getRightEyePoints();
    return cfg;
}
