#ifndef _LOG_LEVEL_FLAG_HPP_
#define _LOG_LEVEL_FLAG_HPP_

#include "spdlog/pattern_formatter.h"


/// @brief Свой логгер для логгирования уровня сообщения
class LogLevelFlag : public spdlog::custom_flag_formatter {
public:
    /// @brief Переопределяем форматирование
    void format(const spdlog::details::log_msg& msg, const std::tm&, spdlog::memory_buf_t& dest) override;
    /// @brief Переопределяем клонирование 
    std::unique_ptr<custom_flag_formatter> clone() const override;
};

#endif //_LOG_LEVEL_FLAG_HPP_