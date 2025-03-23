#ifndef _THREAD_NAME_FLAG_HPP_
#define _THREAD_NAME_FLAG_HPP_

#include "spdlog/pattern_formatter.h"


/// @brief Свой логгер с логгированием имени потока
class ThreadNameFlag : public spdlog::custom_flag_formatter {
public:
    /// @brief Переопределяем форматирование
    void format(const spdlog::details::log_msg& msg, const std::tm&, spdlog::memory_buf_t& dest) override;
    /// @brief Переопределяем клонирование 
    std::unique_ptr<custom_flag_formatter> clone() const override;
};

#endif //_THREAD_NAME_FLAG_HPP