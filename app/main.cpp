//gui
#include "opencv2/highgui.hpp"
///logging
#include "logging/logger.hpp"
///tasks
#include "video_capture.hpp"
#include "tasks/thread_pool.hpp"


int main(int argc, char* argv[])
{
    //Инициализация логгирования
    Logger::init(spdlog::level::debug);
    Logger::set_thread_name("main");
    ThreadPool::init(8);

    spdlog::info("Starting the program");
    //Инициализируем задачи
    VideoCapture capture("C:\\Users\\timur\\Projects\\eye-tracker\\data\\shape_predictor_68_face_landmarks.dat");
    capture.start();
    
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
