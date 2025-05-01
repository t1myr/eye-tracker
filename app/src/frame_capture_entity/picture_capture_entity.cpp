#include "picture_capture_entity.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include "utils/files_proc.hpp"

namespace fs = std::filesystem;

/**
 * @brief Конструктор
 * @param folderPath путь к папке с картинками
 */
PictureCaptureEntity::PictureCaptureEntity(const std::filesystem::path& folderPath)
{
    if (!fs::exists(folderPath) || !fs::is_directory(folderPath)) {
        throw std::invalid_argument("Invalid folder path: " + folderPath.string());
    }

    for (const auto& entry : fs::directory_iterator(folderPath)) {
        if (entry.is_regular_file()) {
            const auto& path = entry.path();
            if (utils::isImageFile(path)) {
                m_imagePaths.emplace_back(path.string());
            }
        }
    }

    // Сортируем по имени
    std::sort(m_imagePaths.begin(), m_imagePaths.end());
}

/**
 * @brief Выдача следующего кадра
 */
cv::Mat PictureCaptureEntity::nextFrame()
{
    if (m_curIndex >= m_imagePaths.size()) {
        return cv::Mat(); // пустой кадр
    }

    cv::Mat image = cv::imread(m_imagePaths[m_curIndex++], cv::IMREAD_COLOR);
    return image;
}

/// @brief Сдвигаем дескриптор на начало
void PictureCaptureEntity::rewind() noexcept
{
    m_curIndex = 0;
}
