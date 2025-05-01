#include "task.hpp"

#include "logging/logger.hpp"

//==================================================================================================
//---------------TASK-------------------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Конструктор
 * @param name имя задачи
 * @param disp дипетчер ивентов
 */
Task::Task(const std::string& name, std::shared_ptr<MessageDispatcher> disp) : m_name(name), m_dispatcher(disp)
{
    static std::atomic<uint32_t> idCounter = 0;
    m_id = idCounter++;
    spdlog::info("Task [{}, id={}] created", m_name, m_id);
}

/**
 * @brief Виртуальный деструктор 
 */
Task::~Task()
{
    stop();
    spdlog::info("Task [{}, id={}] destroyed", m_name, m_id);
}

//==================================================================================================
//---------------Вспомогательные функции------------------------------------------------------------
//==================================================================================================
/**
 * @brief Выдача имени задачи
 * @return const std::string& имя 
 */
const std::string& Task::getName() const noexcept
{
    return m_name;
}

//==============================================================================
/**
 * @brief Выдача id задачи
 * @return const std::string& имя 
 */
TaskId Task::getId() const noexcept
{
    return m_id;
}

//==================================================================================================
//---------------Изменение состояния----------------------------------------------------------------
//==================================================================================================
/**
 * @brief Запускаем задачу
 * @note idle --> active : create thread, change state to active
 * @note suspend -- > active : change state to active
 */
void Task::start()
{
    //Блокируем доступ к состоянию
    std::lock_guard lock(m_state_mutex);

    //Если мы ранее не запускались или уже не запущены
    if(m_state == State::Idle || m_state == State::Suspend)
    {
        if (m_state == State::Suspend) 
        {
            spdlog::info("Task [{}, id={}] resumed", m_name, m_id);
            //Перешли в активное состояние
            m_state = State::Active;
        } else 
        {
            //Перешли в активное состояние
            m_state = State::Active;
            // Создаем новый поток
            m_thread = std::thread(Task::threadFunc, this);
            //зарегистрировали задачу в диспетчере
            m_dispatcher->regTask(m_id, this);
        }

    }
}

/**
 * @brief Останавливаем задачу
 * @note suspend, active --> idle: thread joined, task unreg
 * @note others : no effect
 */
void Task::stop()
{
    {
        //Блокируем доступ к состоянию
        std::lock_guard lock(m_state_mutex); 
        //Если уже остановлены, можем выйти
        if(m_state == State::Idle) return;
        //Переходим в ожидание остановки
        m_state = State::WaitStop;
    }

    {
        //Блокируем поток и обрабатываем оставшиеся сообщения
        std::lock_guard lock(messageQueue.m);
        handleTemporaryQueue(messageQueue.q);
    }
    
    // Заканчиваем поток
    if (m_thread.joinable())
        m_thread.join();

    //Убираем задачу из диспетчера
    m_dispatcher->unregTask(m_id);
    //Уходим в простой
    m_state = State::Idle;
    spdlog::info("Task [{}, id={}] stopped", m_name, m_id);
}

/**
 * @brief Отправляем задачу в сон
 * @note active --> suspend : state changed
 * @note other : not effect
 */
void Task::suspend()
{
    std::lock_guard lock(m_state_mutex);

    if (m_state == State::Active) {
        m_state = State::Suspend;
        spdlog::info("Task [{}, id={}] suspended", m_name, m_id);
    }
}

//==================================================================================================
//---------------Работа с сообщениями---------------------------------------------------------------
//==================================================================================================
/**
 * @brief Отправляем сообщение в другую задачу
 * @param msg сообщение
 */
void Task::sendMessage(Message&& msg) const noexcept
{
    m_dispatcher->pollMessage(std::move(msg));
}

//==================================================================================================
//---------------Работа с таймерами-----------------------------------------------------------------
//==================================================================================================
/**
 * @brief Задаем таймер
 * @param id айди 
 * @param timeout таймаут 
 * @param periodic периодичность
 */
void Task::setTimer(uint32_t id, std::chrono::milliseconds timeout, bool periodic)
{
    std::lock_guard lock(m_timerMutex);
    m_timers[id] = {
        .nextFireTime = std::chrono::steady_clock::now() + timeout,
        .interval = timeout,
        .periodic = periodic,
        .active = true
    };
}

/**
 * @brief Сброс таймера
 * @param id айди 
 */
void Task::resetTimer(uint32_t id)
{
    std::lock_guard lock(m_timerMutex);
    auto it = m_timers.find(id);
    if (it != m_timers.end())
    {
        it->second.active = false;
    }
}

//==================================================================================================
//---------------Работа с потоком-------------------------------------------------------------------
//==================================================================================================
/**
 * @brief Основная потоковая функция
 * @param t указатель на задачу
 */
void Task::threadFunc(Task* t)
{
    //установили имя потока
    Logger::set_thread_name(t->m_name);
    spdlog::info("Task [{}, id={}] started", t->m_name, t->m_id);
    t->init();

    //запустили основную функцию
    while (t->m_state >= State::Suspend) {
        {
            std::unique_lock lock(t->m_state_mutex);
            if (t->m_state == State::WaitStop) break;  // Ожидаем остановки
            if (t->m_state == State::Suspend) {
                lock.unlock();
                // Если задача приостановлена, пропускаем основной цикл
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        }

        //Выполняем работу по таймерам
        t->handleTimers();
        //Выполняем пользовательскую работу
        try
        {
            t->mainFunc();
        }catch(const std::exception& exc)
        {
            spdlog::critical("Uncathed exception occuried in user work : {}", exc.what());
        }

        std::queue<Message> tempQueue;
        {
            //Блокируем доступ к очереди
            std::lock_guard lock(t->messageQueue.m);
            //Забираем сообщения из распределенной очереди в локальную
            tempQueue.swap(t->messageQueue.q);
        }
        //Обрабатываем сообщения
        t->handleTemporaryQueue(tempQueue);
        ///Спим 10 мс
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

//==================================================================================================
//---------------Работа с сообщениями---------------------------------------------------------------
//==================================================================================================
/**
 * @brief Обработка временной очереди
 * @param queue очередь
 */
void Task::handleTemporaryQueue(std::queue<Message>& queue) noexcept
{
    //Проходимся по всей очереди
    while(!queue.empty())
    {
        try
        {
            //Обратка сообщения
            receiveMessage(std::move(queue.front()));    
        }catch(const std::exception& exc)
        {
            spdlog::critical("Uncatched exception occuried during work"
                                                "on message : {}", exc.what());
        }
        queue.pop();
    }
}

//==============================================================================
//---------------Работа с таймерами---------------------------------------------
//==============================================================================
/**
 * @brief Обрабатываем таймера
 */
void Task::handleTimers()
{
    auto now = std::chrono::steady_clock::now();
    std::lock_guard lock(m_timerMutex);

    //Проходимся по всем таймерам
    for (auto& [id, timer] : m_timers)
    {
        //Рассматриваем только активные или те, что вышли
        if (timer.active && now >= timer.nextFireTime)
        {
            try
            {
                //Обработка события по таймеру
                onTimer(id);
            }catch(const std::exception& exc)
            {
                spdlog::critical("Uncatched exception occuried during work"
                                        "on timer id={} : {}", id, exc.what());
            }
            
            if (timer.periodic)
                //Если периодичный - обновляем время
                timer.nextFireTime = now + timer.interval;
            else
                //Помечаем таймер как неактивный
                timer.active = false;
        }
    }
}
