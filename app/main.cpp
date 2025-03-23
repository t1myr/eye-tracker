#include <iostream>
#include "logging/logger.hpp"
#include "video_capture.hpp"
#include "tasks/dummy_task.hpp"
#include "tasks/super_dummy_task.hpp"


int main(int argc, char* argv[])
{
    Logger::init(spdlog::level::debug);
    Logger::set_thread_name("main");
    spdlog::info("Starting the program");
    SuperDummyTask sdTask;
    DummyTask task;
    sdTask.start();
    task.start();

    spdlog::info("thread sleeping");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    sdTask.suspend();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    sdTask.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    sdTask.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    sdTask.stop();

    do
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }while(true);
    
    return 0;
}
