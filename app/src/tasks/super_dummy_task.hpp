#ifndef _SUPER_DUMMY_TASK_HPP_
#define _SUPER_DUMMY_TASK_HPP_

#include "logging/logger.hpp"
#include "task.hpp"


class SuperDummyTask : public Task
{
public:
    SuperDummyTask() : Task("SupDummy", MessageDispatcher::get())
    {

    }

    /**
     * @brief Принимаем сообщение от другой задачи
     * @param msg сообщение
     */
    void receiveMessage(Message &&msg) noexcept override
    {
        spdlog::info("Super dummy task received message srcId={}, dstId={}", msg.src, msg.dst);
    }
};

#endif //_SUPER_DUMMY_TASK_HPP_