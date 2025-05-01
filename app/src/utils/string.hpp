#ifndef _STRING_HPP_
#define _STRING_HPP_

#include <string>

namespace utils
{
    /// @brief Приводим строку к нижнему регистру с копированием
    std::string toLowerCopy(const std::string& str);

    /// @brief Приводим строку к нижнему регистру на месте
    void toLowerInplace(std::string& str);
}

#endif // _STRING_HPP_