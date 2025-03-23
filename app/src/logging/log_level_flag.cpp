#include "log_level_flag.hpp"
#include "logger.hpp"

/// @brief Переопределяем форматирование
void LogLevelFlag::format(const spdlog::details::log_msg& msg, const std::tm&, spdlog::memory_buf_t& dest)
{
    std::string level = spdlog::level::to_string_view(msg.level).data();
    std::string bracketed = "[" + level + "]";
    bracketed += std::string(std::max(0, 11 - (int)bracketed.size()), ' ');
    dest.append(bracketed.data(), bracketed.data() + bracketed.size());
}
/// @brief Переопределяем клонирование 
std::unique_ptr<spdlog::custom_flag_formatter> LogLevelFlag::clone() const
{
    return spdlog::details::make_unique<LogLevelFlag>();
}