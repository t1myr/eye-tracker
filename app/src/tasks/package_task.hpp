#ifndef _PACKAGE_TASK_HPP_
#define _PACKAGE_TASK_HPP_

#include "task.hpp"
#include <future>


/// @brief Класс назначаемой задачи
class PackageTask : public Task
{
public:
    struct ExecuteBody
    {
        virtual ~ExecuteBody() {}
    };

    /// @brief Конструктор
    PackageTask(const std::string& name, std::shared_ptr<MessageDispatcher> disp);

    /// @brief Назначение задачи с ожиданием её завершения
    std::future<std::unique_ptr<ExecuteBody>> post(std::unique_ptr<ExecuteBody>&& body);

protected:
    /// @brief Запускаем расчет задачи
    virtual std::unique_ptr<ExecuteBody> execute(std::unique_ptr<ExecuteBody>&& body) = 0;

private:
    /// @brief Основная функция задачи
    void mainFunc() override;

    struct TaskItem {
        std::unique_ptr<ExecuteBody> body;
        std::promise<std::unique_ptr<ExecuteBody>> done;
    };

    std::queue<TaskItem> m_queue;
    std::mutex m_queueMutex;
};

#endif