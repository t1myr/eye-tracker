//gui
#include "opencv2/highgui.hpp"
///logging
#include "logging/logger.hpp"

///tasks
#include "control_task.hpp"
#include "virtual_scene/virtual_scene_task.hpp"
#include "input_control/input_control_task.hpp"
#include "mouse_control/mouse_control_task.hpp"
#include "tasks/thread_pool.hpp"

//argparse
#include "argparse/argparse.hpp"

static constexpr std::size_t kMaxThreadPoolSize = 8;
// "C:\\Users\\timur\\Projects\\eye-tracker\\data\\calibrate"

int main(int argc, char* argv[])
{
    //Инициализируем парсер аргументов
    argparse::ArgumentParser program("eye_tracker");

    //Аргумент логгирования
    program.add_argument("-l", "--logging")
    .help("Logging level")
    .scan<'i', int>()
    .default_value(static_cast<int>(spdlog::level::info));

    //Аргумент конфиг
    program.add_argument("-p", "--predictor")
    .help("Path to shape predictor")
    .required();

    //Парсим аргументы
    try {
       program.parse_args(argc, argv);   
    }
    catch (const std::exception& err) {
        std::cout << "Cannot parse some argument" << std::endl;
        std::cout << program << std::endl;
        std::exit(1);
    }

    auto logLevel = program.get<int>("-l");
    auto predictorPath = program.get<std::string>("--predictor");

    if(logLevel < spdlog::level::trace || logLevel >= spdlog::level::off)
    {
        std::cout << program << std::endl;
        std::exit(1);
    }

    //Инициализация логгирования
    Logger::init(static_cast<spdlog::level::level_enum>(logLevel));
    Logger::set_thread_name("main");
    ThreadPool::init(kMaxThreadPoolSize);
    spdlog::info("Starting the program");
    //Инициализируем задачи
    ControlTask ctrl(predictorPath);
    VirtualScene scene;
    InputControlTask input;
    MouseControlTask mouse(MouseControlTask::Mode::kVisual);

    //Задаем взаимодействие между задачами
    ctrl.setVirtualScene(&scene);
    scene.setCtrl(&ctrl);

    //Запускаем
    ctrl.start();
    scene.start();
    input.start();
    mouse.start();
    
    while(true)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return 0;
}
