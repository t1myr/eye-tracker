#include "mouse_control_task.hpp"
  
//==================================================================================================
//---------------MOUSE CONTROL TASK-----------------------------------------------------------------
//==================================================================================================
/// @brief Конструктор
MouseControlTask::MouseControlTask(Mode mode) : 
    Task("mouse", MessageDispatcher::get()), 
    m_curMode(mode)
{
    
}

//==================================================================================================
//---------------Работа с потоком-------------------------------------------------------------------
//==================================================================================================
/// @brief Основная функция задачи
void MouseControlTask::mainFunc()
{

}

/// @brief Инициализация в потоке
void MouseControlTask::init() 
{

}

//==================================================================================================
//---------------Работа с сообщениями---------------------------------------------------------------
//==================================================================================================
/// @brief Принимаем сообщение от другой задачи
void MouseControlTask::receiveMessage(Message &&msg)
{

}

//==================================================================================================
//---------------Работа с отображением---------------------------------------------------------------
//==================================================================================================
/// @brief Отобразить точку 
void MouseControlTask::display() const noexcept
{

}

/// @brief Задать позицию мышки
void MouseControlTask::setMousePose() const noexcept
{

}
