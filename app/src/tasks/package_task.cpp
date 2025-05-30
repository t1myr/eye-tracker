#include "package_task.hpp"


/// @brief Конструктор
PackageTask::PackageTask(const std::string& name, std::shared_ptr<MessageDispatcher> disp) : 
    Task(name, disp)
{
    start(); // поток создаётся сразу
}

/// @brief Назначение задачи с ожиданием её завершения
std::future<std::unique_ptr<PackageTask::ExecuteBody>> PackageTask::post(std::unique_ptr<ExecuteBody>&& body)
{
    TaskItem item;
    item.body = std::move(body);
    std::future<std::unique_ptr<ExecuteBody>> future = item.done.get_future();
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_queue.push(std::move(item));
    }

    return future;
}

/// @brief Основная функция задачи
void PackageTask::mainFunc()
{
    TaskItem item;
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        if (m_queue.empty()) return;
        item = std::move(m_queue.front());
        m_queue.pop();
    }

    try {
        auto result = execute(std::move(item.body));
        item.done.set_value(std::move(result));
    } catch (...) {
        item.done.set_exception(std::current_exception());
    }
}
