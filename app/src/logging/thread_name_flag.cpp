#include "thread_name_flag.hpp"
#include "logger.hpp"

/// @brief Переопределяем форматирование
void ThreadNameFlag::format(const spdlog::details::log_msg& msg, const std::tm&, spdlog::memory_buf_t& dest)
{
    std::string name = Logger::get_thread_name();
    std::string result = "[" + name + "]";

    if(result.length() < 10)
        result.append(10 - result.length(), ' ');

    dest.append(result.data(), result.data() + result.size());
}
/// @brief Переопределяем клонирование 
std::unique_ptr<spdlog::custom_flag_formatter> ThreadNameFlag::clone() const
{
    return spdlog::details::make_unique<ThreadNameFlag>();
}