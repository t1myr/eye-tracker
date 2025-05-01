#ifndef _PICTURE_CAPTURE_ENTITY_HPP_
#define _PICTURE_CAPTURE_ENTITY_HPP_

#include <filesystem>

#include "frame_capture_entity.hpp"

/// @brief Захват кадра
class PictureCaptureEntity : public FrameCaptureEntity
{
public:
    /// @brief Конструктор
    PictureCaptureEntity(const std::filesystem::path& folderPath);

    /// @brief Выдача следующего кадра
    cv::Mat nextFrame() override;

    /// @brief Сдвигаем дескриптор на начало
    void rewind() noexcept;

private:
    std::vector<std::string> m_imagePaths;
    std::size_t m_curIndex;
};

#endif // _PICTURE_CAPTURE_ENTITY_HPP_
