#ifndef _DUMMY_TASK_HPP_
#define _DUMMY_TASK_HPP_

#include "logging/logger.hpp"
#include "task.hpp"


class DummyTask : public Task
{
public:
    DummyTask() : Task("Dummy", MessageDispatcher::get())
    {

    }

    /**
     * @brief Принимаем сообщение от другой задачи
     * @param msg сообщение
     */
    void receiveMessage(Message &&msg) noexcept override
    {
        spdlog::info("Dummy task received message srcId={}, dstId={}", msg.src, msg.dst);
    }   
};

#endif //_DUMMY_TASK_HPP_