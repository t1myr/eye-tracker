//gui
#include "opencv2/highgui.hpp"
///logging
#include "logging/logger.hpp"
///tasks
#include "control_task.hpp"
#include "virtual_scene_task.hpp"
#include "tasks/thread_pool.hpp"


static constexpr std::size_t kMaxThreadPoolSize = 8;


int main(int argc, char* argv[])
{
    //Инициализация логгирования
    Logger::init(spdlog::level::info);
    Logger::set_thread_name("main");
    ThreadPool::init(kMaxThreadPoolSize);

    spdlog::info("Starting the program");
    //Инициализируем задачи
    ControlTask ctrl("C:\\Users\\timur\\Projects\\eye-tracker\\data\\shape_predictor_68_face_landmarks.dat");
    VirtualScene scene;
    ctrl.setVirtualScene(&scene);
    scene.setCtrl(&ctrl);

    ctrl.start();
    scene.start();
    
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
