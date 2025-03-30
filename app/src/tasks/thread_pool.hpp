#ifndef _THREAD_POOL_HPP_
#define _THREAD_POOL_HPP_

#include <vector>
#include <queue>
#include <thread>
#include <future>
#include <functional>
#include <atomic>
#include <condition_variable>

/**
 * @brief Пул потоков
 */
class ThreadPool {
public:

    // Запрещаем копирование и перемещение
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    /**
     * @brief Деструктор
     */
    ~ThreadPool();
    
    /**
     * @brief Инициализация пула
     * @param numThreads 
     */
    static void init(std::size_t numThreads);

    /**
     * @brief Получаем экземпляр пула
     * @return ThreadPool* 
     */
    static ThreadPool* get();

    /**
     * @brief Добавляем задачу в очередь
     * @tparam F 
     * @tparam Args
     * @param f callable объект, функция будет вызвана в отдельном потоке
     * @param args аргументы
     * @return std::future<std::invoke_result_t<F, Args...>> future
     */
    template<typename F, typename... Args>
    auto enqueue(F&& f, Args&&... args) -> std::future<std::invoke_result_t<F, Args...>>;

    /**
     * @brief Ставим набор в задач в очередь на синхронное исполнение
     * @tparam F 
     * @tparam Args 
     * @param m_tasks задачи
     * @return std::vector<std::future<void>> 
     */
    template<typename F, typename... Args>
    auto enqueueAcquire(std::vector<std::function<void()>> tasks) -> std::vector<std::future<void>>;

    /**
     * @brief Выдача количества свободных исполнителей
     * @return std::size_t 
     */
    std::size_t getFreeWorkersNum();

private:
    std::vector<std::thread> m_workers;
    std::queue<std::function<void()>> m_tasks;

    std::mutex m_queueMutex;
    std::condition_variable m_condition;
    std::atomic<bool> m_stop;
    std::size_t m_busyWorkers;

    static std::unique_ptr<ThreadPool> m_instance;

    /**
     * @brief Конструктор
     * @param numThreads Количество потоков в пуле
     */
    ThreadPool(std::size_t numThreads);
};
    
template<typename F, typename... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) -> std::future<std::invoke_result_t<F, Args...>> {
    using return_type = std::invoke_result_t<F, Args...>;

    //Создаем задачу
    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    //Добавляем задачу в очередь
    std::future<return_type> res = task->get_future();
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_tasks.emplace([task]() { (*task)(); });
    }
    //Оповещаем об этом
    m_condition.notify_one();
    return res;
}
    
template<typename F, typename... Args>
auto ThreadPool::enqueueAcquire(std::vector<std::function<void()>> tasks) -> std::vector<std::future<void>> 
{
    std::vector<std::future<void>> futures;
    {
        auto numThreads = tasks.size();

        std::unique_lock<std::mutex> lock(m_queueMutex);

        // Ждём, пока освободится нужное количество потоков
        m_condition.wait(lock, [this, numThreads] {
            return m_workers.size() - m_busyWorkers >= numThreads;
        });

        // "Бронируем" потоки
        m_busyWorkers += tasks.size();

        // Добавляем задачи в очередь
        for (auto& task : tasks) {
            auto packagedTask = std::make_shared<std::packaged_task<void()>>(task);
            futures.push_back(packagedTask->get_future());

            this->m_tasks.emplace([this, packagedTask, numThreads]() {
                (*packagedTask)();
                
                // Освобождаем потоки после выполнения
                {
                    std::lock_guard<std::mutex> lock(m_queueMutex);
                    m_busyWorkers--;
                }
                m_condition.notify_all();
            });
        }
    }

    m_condition.notify_all();
    return futures;
}

#endif //_THREAD_POOL_HPP_