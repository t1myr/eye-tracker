#include "message_dispatcher.hpp"
#include "task.hpp"

/**
 * @brief Добавляем задачу
 * @param taskId id задачи
 * @param t задача
 */
void MessageDispatcher::regTask(TaskId taskId, Task* t) noexcept
{
    if(tasks.contains(taskId))
    {
        spdlog::error("Dispatcher already contains task {}", t->getName());
        return;
    }
    spdlog::debug("Dispatcher add task {} with id={}", t->getName(), t->m_id);
    tasks[taskId] = t;
}

/**
 * @brief Убираем задачу из диспетчера
 * @param taskId id задачи
 */
void MessageDispatcher::unregTask(TaskId taskId) noexcept
{
    if(!tasks.contains(taskId))
    {
        spdlog::error("Dispatcher doesnt have task with id={}, nothing to delete", taskId);
        return;
    }
    spdlog::debug("Dispatcher delete task with id={}", taskId);
    tasks.erase(taskId);
}

/**
 * @brief Получаем экземпляр диспетчера
 */
std::shared_ptr<MessageDispatcher> MessageDispatcher::get()
{
    if(!m_instance)
    {
        m_instance = std::make_shared<MessageDispatcher>();
    }
    return m_instance;
}

/**
 * @brief Добавляем сообщение в очередь задачи
 * @param msg сообщение
 */
void MessageDispatcher::pollMessage(Message&& msg) const noexcept
{
    if(tasks.contains(msg.dst))
    {
        std::unique_lock lock(tasks.at(msg.dst)->messageQueue.m);
        tasks.at(msg.dst)->messageQueue.q.push(std::move(msg));
    }else
    {
        spdlog::error("Event dispatcher doesnt have task with id={}", msg.dst);
    }
}

std::shared_ptr<MessageDispatcher> MessageDispatcher::m_instance = nullptr;
