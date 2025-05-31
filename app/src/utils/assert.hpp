#ifndef _ASSERT_HPP_
#define _ASSERT_HPP_

#include <cstdlib>
#include <string_view>

/// @brief Проверяем выражение
void assert_printf(bool expr, std::string_view file, int line, std::string_view msg);

/// @brief Макрос ассерта
#define ASSERT_PRINTF(expr, ...) \
    ((expr) ? void(0) : assert_printf(false, __FILE__, __LINE__, fmt::format(__VA_ARGS__)))

#endif