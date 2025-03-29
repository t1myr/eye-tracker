#ifndef _TASK_HPP_
#define _TASK_HPP_

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include "message_dispatcher.hpp"


/// @brief Класс-обертка над потоком
class Task
{
public:
    /**
     * @brief Конструктор
     * @param name имя задачи
     * @param disp дипетчер ивентов
     */
    Task(const std::string &name, std::shared_ptr<MessageDispatcher> disp);

    /**
     * @brief Конструктор копирования удаляем - потоки не копируются
     * @param other 
     */
    Task(const Task& other) = delete;

    /// @brief Виртуальный деструктор 
    virtual ~Task();

    //---------------Вспомогательные функции------------------------------------
    /**
     * @brief Выдача имени задачи
     * @return const std::string& имя 
     */
    const std::string& getName() const noexcept;

    /**
     * @brief Выдача id задачи
     * @return const std::string& имя 
     */
    TaskId getId() const noexcept;

    //---------------Изменение состояния----------------------------------------
    /// @brief Запускаем задачу
    void start();
    /// @brief Останавливаем задачу
    void stop();
    /// @brief Отправляем задачу в сон
    void suspend();

protected:
    //---------------Работа с потоком-------------------------------------------
    /**
     * @brief Основная функция задачи
     * @note Переопределяется в дочернем классе, выполняется один раз за тик 
     * основного цикла
     */
    virtual void mainFunc() {}
    
    //---------------Работа с сообщениями---------------------------------------
    /**
     * @brief Отправляем сообщение в другую задачу
     * @param msg сообщение
     */
    void sendMessage(Message &&msg) const noexcept;

    /**
     * @brief Принимаем сообщение от другой задачи
     * @param msg сообщение
     */
    virtual void receiveMessage(Message &&msg) noexcept = 0;

    //---------------Работа с таймерами-----------------------------------------
    /**
     * @brief Обрабатываем событие по таймеру
     * @param id айди таймера
     */
    virtual void onTimer(uint32_t id) {}
    /**
     * @brief Задаем таймер
     * @param id айди 
     * @param timeout таймаут 
     * @param periodic периодичность
     */
    void setTimer(uint32_t id, std::chrono::milliseconds timeout, bool periodic);
    /**
     * @brief Сброс таймера
     * @param id айди таймера
     */
    void resetTimer(uint32_t id);

private:
    //---------------Работа с состоянием----------------------------------------
    /// @brief Состояния задачи 
    enum class State
    {
        Idle,
        WaitStop,
        Suspend,
        Active
    };
    //Состояние задачи, не блокируем там, где можем
    std::atomic<State> m_state{State::Idle}; 
    std::mutex m_state_mutex; //Мьютекс для безопаснго доступа к состоянию

    //---------------Работа с потоком-------------------------------------------
    /**
     * @brief Основная потоковая функция
     * @param t указатель на задачу
     */
    static void threadFunc(Task *t);

    std::thread m_thread; //Поток, в котором работает задача
    std::string m_name; //Имя задачи
    TaskId m_id; //Id задачи

    //---------------Работа с сообщениями---------------------------------------
    /**
     * @brief Обработка временной очереди
     * @param queue очередь
     */
    void handleTemporaryQueue(std::queue<Message>& queue) noexcept;
    
    //Диспетчер сообщений
    std::shared_ptr<MessageDispatcher> m_dispatcher;
    //Очередь сообщений
    struct
    {
        std::queue<Message> q;
        std::mutex m;
    } messageQueue;

    //---------------Работа с таймерами-----------------------------------------
    /// @brief Обрабатываем таймера
    void handleTimers();

    /// @brief Собственно таймер
    struct Timer
    {
        std::chrono::steady_clock::time_point nextFireTime;
        std::chrono::milliseconds interval;
        bool periodic;
        bool active;
    };
    std::unordered_map<uint32_t, Timer> m_timers; /// таймера
    std::mutex m_timerMutex; /// мьютекс для таймеров

    friend class MessageDispatcher;
};

#endif //_TASK_HPP_