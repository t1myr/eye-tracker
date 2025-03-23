#ifndef _LOGGER_HPP_
#define _LOGGER_HPP_

#include <memory>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "fmt_thread_id.hpp"

/// @brief Класс, реализующий обертку над spdlog
class Logger
{
public:
    /// @brief Запрещаем копирование
    Logger(const Logger& other) = delete;
    /// @brief Запрещаем перемещение
    Logger(Logger&& other) = delete;

    /// @brief Получаем экземпляр логгера
    static std::shared_ptr<spdlog::logger> get() noexcept;
    /// @brief Инициализация
    static void init(spdlog::level::level_enum level) noexcept;
    /// @brief Получаем имя текущего потока
    static std::string get_thread_name() noexcept;
    /// @brief Ставим имя текущего потока
    static void set_thread_name(const std::string& name) noexcept;
private:
    /// @brief Конструктор логгера
    Logger(spdlog::level::level_enum level) noexcept;

    //File sinks
    std::shared_ptr<spdlog::sinks::stderr_color_sink_mt> m_console_sink;
    std::shared_ptr<spdlog::sinks::basic_file_sink_mt> m_file_sink;

    //Logger
    static inline std::shared_ptr<spdlog::logger> m_logger;

    //Thread names and mutex
    static std::unordered_map<std::thread::id, std::string> m_thread_names;
    static std::mutex m_thread_mutex;
};

#endif //_LOGGER_HPP_