#include "message_dispatcher.hpp"
#include "task.hpp"

/**
 * @brief Добавляем задачу
 * @param taskId id задачи
 * @param t задача
 */
void MessageDispatcher::regTask(TaskId taskId, Task* t)
{
    if(tasks.contains(taskId))
    {
        throw std::runtime_error(std::format("Dispatcher already contains task {}", t->getName()));
    }
    spdlog::debug("Dispatcher add task {} with id={}", t->getName(), t->m_id);
    tasks[taskId] = t;
}

/**
 * @brief Убираем задачу из диспетчера
 * @param taskId id задачи
 */
void MessageDispatcher::unregTask(TaskId taskId)
{
    if(!tasks.contains(taskId))
    {
        throw std::runtime_error(std::format("Dispatcher doesnt have task with id={}, nothing to delete", taskId));
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
void MessageDispatcher::pollMessage(Message&& msg) const
{
    if(tasks.contains(msg.dst))
    {
        std::unique_lock lock(tasks.at(msg.dst)->messageQueue.m);
        tasks.at(msg.dst)->messageQueue.q.push(std::move(msg));
    }else
    {
        throw std::runtime_error(std::format("Event dispatcher doesnt have task with id={}", msg.dst));
    }
}

std::shared_ptr<MessageDispatcher> MessageDispatcher::m_instance = nullptr;
