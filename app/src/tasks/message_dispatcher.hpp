#ifndef _MESSAGE_DISPATCHER_HPP
#define _MESSAGE_DISPATCHER_HPP

#include <thread>
#include <memory>
#include <queue>

#include "logging/logger.hpp"

/// @brief Пустое тело сообщения с виртуальным деструктором
struct MessageBody
{   
    virtual ~MessageBody() {}
};

/// @brief Сообщение для межпоточного взаимодействия
struct Message
{
    uint32_t src;
    uint32_t dst;
    std::unique_ptr<MessageBody> body;
};

//Task forward declaration
class Task;

/// @brief Диспетчер сообщений
class MessageDispatcher
{
public:
    /**
     * @brief Добавляем задачу
     * @param uint32_t id задачи
     * @param t задача
     */
    void regTask(uint32_t id, Task* t) noexcept;

    /**
     * @brief Убираем задачу из диспетчера
     * @param uint32_t id задачи
     */
    void unregTask(uint32_t id) noexcept;
    
    /**
     * @brief Получаем экземпляр диспетчера
     */
    static std::shared_ptr<MessageDispatcher> get();

    /**
     * @brief Добавляем сообщение в очередь задачи
     * @param msg сообщение
     */
    void pollMessage(Message&& msg) const noexcept;

private:
    static std::shared_ptr<MessageDispatcher> m_instance;
    std::unordered_map<uint32_t, Task*> tasks;
};

#endif //_MESSAGE_DISPATCHER_HPP