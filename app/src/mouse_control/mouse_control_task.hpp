#ifndef _MOUSE_CONTROL_HPP_
#define _MOUSE_CONTROL_HPP_

#include "../tasks/task.hpp"

/// @brief Задача по управлению отображения координат курсора
class MouseControlTask : public Task
{
public:

    /// @brief Режим отображения мышки
    enum class Mode
    {
        kUknown,
        kVisual,
        kMouse,
    };

    /// @brief Конструктор
    MouseControlTask(Mode mode);

private:
    //---------------Работа с потоком-------------------------------------------
    /// @brief Основная функция задачи
    void mainFunc() override;

    /// @brief Инициализация в потоке
    void init() override;

    //---------------Работа с сообщениями---------------------------------------
    /// @brief Принимаем сообщение от другой задачи
    void receiveMessage(Message &&msg) override;

    //---------------Работа с отображением--------------------------------------
    std::mutex m_modeMutex;
    Mode m_curMode {Mode::kUknown}; ///Режим работы

    /// @brief Отобразить точку 
    void display() const noexcept;

    void setMousePose() const noexcept;
};

#endif