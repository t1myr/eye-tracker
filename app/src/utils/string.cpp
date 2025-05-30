#include "string.hpp"

#include <algorithm>
#include <iterator>


namespace utils
{
    /// @brief Приводим строку к нижнему регистру с копированием
    std::string toLowerCopy(const std::string& str)
    {
        std::string res;
        std::transform(str.begin(), str.end(), std::back_inserter(res),
                    [](unsigned char c) { return std::tolower(c); });
        return res;
    }

    /// @brief Приводим строку к нижнему регистру на месте
    void toLowerInplace(std::string& str)
    {
        std::transform(str.begin(), str.end(), str.begin(),
                    [](unsigned char c) { return std::tolower(c); });
    }
}
