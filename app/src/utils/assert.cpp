#include "assert.hpp"

/// @brief Проверяем выражение
void assert_printf(bool expr, std::string_view file, int line, std::string_view msg)
{
    if (!expr)
    {
        spdlog::error("Assertion failed: {}, file {}, line {}", msg, file, line);
        std::abort();
    }
}
