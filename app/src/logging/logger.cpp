#include "logger.hpp"
#include "opencv2/core/utils/logger.hpp"
#include "thread_name_flag.hpp"
#include "log_level_flag.hpp"

//Thread names and mutex
std::unordered_map<std::thread::id, std::string> Logger::m_thread_names;
std::mutex Logger::m_thread_mutex;

/**
 * @brief Получаем экземпляр логгера
 * @return std::shared_ptr<spdlog::logger>
 */
std::shared_ptr<spdlog::logger> Logger::get() noexcept
{
    return m_logger;
}

/**
 * @brief Инициализация
 * @param level уровень логгирования
 */
void Logger::init(spdlog::level::level_enum level) noexcept
{
    if(!m_logger)
    {
        Logger instance(level);
    }
}

/**
 * @brief Получаем имя текущего потока
 * @return std::string 
 */
std::string Logger::get_thread_name() noexcept
{
    std::lock_guard<std::mutex> lock(m_thread_mutex);
    auto it = m_thread_names.find(std::this_thread::get_id());
    return (it != m_thread_names.end()) ? it->second : "unknown";
}

/**
 * @brief Задаем имя текущего потока
 * @param name имя потока
 */
void Logger::set_thread_name(const std::string& name) noexcept{
    std::lock_guard<std::mutex> lock(m_thread_mutex);
    m_thread_names[std::this_thread::get_id()] = name;
}

/**
 * @brief Конструктор логгера
 * @param level уровень логгирования
 */
Logger::Logger(spdlog::level::level_enum level) noexcept : 
            m_console_sink(std::make_shared<spdlog::sinks::stderr_color_sink_mt>()),
            m_file_sink(std::make_shared<spdlog::sinks::basic_file_sink_mt>("log.txt", true))
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    auto formatter = std::make_unique<spdlog::pattern_formatter>();
    formatter->add_flag<ThreadNameFlag>('N'); 
    formatter->add_flag<LogLevelFlag>('L'); 
    formatter->set_pattern("[%Y-%m-%d %H:%M:%S.%e] %N %^%L%$ %v");

    m_logger = std::make_shared<spdlog::logger>("multi_logger", spdlog::sinks_init_list{m_console_sink, m_file_sink});
    m_logger->set_level(level);

    m_logger->set_formatter(std::move(formatter));
    spdlog::set_default_logger(m_logger);
}
