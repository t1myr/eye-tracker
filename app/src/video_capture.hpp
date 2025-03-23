#ifndef _VIDEO_CAPTURE_HPP_
#define _VIDEO_CAPTURE_HPP_

#include <thread>
#include <mutex>
#include "opencv2/videoio.hpp"


class VideoCapture
{
public:
    VideoCapture();

    virtual ~VideoCapture();

    bool open();

    void start();
    
private:
    /// API камеры
    int m_deviceId;
    int m_apiId;
    cv::VideoCapture m_cap;

    //Потоковое взаимодействие
    std::thread m_thread;
    std::mutex m_mutex;
};

#endif //_VIDEO_CAPTURE_HPP_

