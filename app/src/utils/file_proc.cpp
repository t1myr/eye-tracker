#include "files_proc.hpp"

#include "string.hpp"

namespace fs = std::filesystem;

namespace utils
{
    /**
     * @brief Проверяем что файл является картинкой
     * @param path путь к файлу
     * @return bool 
     */
    bool isImageFile(const fs::path& path)
    {
        auto ext = path.extension().string();
        auto lowercaseExt = utils::toLowerCopy(ext);
        return lowercaseExt == ".jpg" || lowercaseExt == ".jpeg" ||
               lowercaseExt == ".png" || lowercaseExt == ".bmp" ||
               lowercaseExt == ".tiff";
    }
}
