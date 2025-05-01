#ifndef _FILES_PROC_HPP_
#define _FILES_PROC_HPP_

#include <filesystem>
#include <string>

namespace utils
{
    /// @brief Проверяем что файл является картинкой
    bool isImageFile(const std::filesystem::path& path);
}

#endif // _FILES_PROC_HPP_
