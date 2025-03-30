#include "thread_pool.hpp"

#include <cassert>


std::unique_ptr<ThreadPool> ThreadPool::m_instance = nullptr;

/**
 * @brief Деструктор
 */
ThreadPool::~ThreadPool() {
    m_stop = true;
    m_condition.notify_all();
    for (std::thread& worker : m_workers)
        worker.join();
}

/**
 * @brief Инициализация пула
 * @param numThreads 
 */
void ThreadPool::init(std::size_t numThreads)
{
    if(!m_instance)
        m_instance.reset(new ThreadPool(numThreads));
}

/**
 * @brief Получаем экземпляр пула
 * @return ThreadPool* 
 */
ThreadPool* ThreadPool::get()
{
    assert(m_instance != nullptr && "Instance must be initialized before");
    return m_instance.get();
}
    
/**
 * @brief Выдача количества свободных исполнителей
 * @return std::size_t 
 */
size_t ThreadPool::getFreeWorkersNum() {
    std::lock_guard<std::mutex> lock(m_queueMutex);
    return m_workers.size() - m_busyWorkers;
}

/**
 * @brief Конструктор
 * @param numThreads 
 */
ThreadPool::ThreadPool(size_t numThreads) : m_stop(false), m_busyWorkers(0) {
    for (size_t i = 0; i < numThreads; ++i) {
        m_workers.emplace_back([this] {
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(m_queueMutex);
                    m_condition.wait(lock, [this] { return m_stop || !m_tasks.empty(); });
                    if (m_stop && m_tasks.empty()) return;
                    task = std::move(m_tasks.front());
                    m_tasks.pop();
                    ++m_busyWorkers;
                }
                task();
                {
                    std::lock_guard<std::mutex> lock(m_queueMutex);
                    --m_busyWorkers;
                }
                m_condition.notify_all();
            }
        });
    }
}
