cmake_minimum_required(VERSION 3.20)

project(eye_tracker VERSION 1.0 LANGUAGES CXX)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS  "src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS  "src/*.hpp")

# opencv
find_package(OpenCV CONFIG REQUIRED)
find_package(dlib CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
include_directories(src)


add_library(eye_tracker_lib STATIC ${HEADERS} ${SOURCES})
target_include_directories(eye_tracker_lib PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(eye_tracker_lib ${OpenCV_LIBS} dlib::dlib spdlog::spdlog)

set_target_properties(eye_tracker_lib PROPERTIES LINKER_LANGUAGE CXX)

add_executable(eye_tracker main.cpp)
target_link_libraries(eye_tracker eye_tracker_lib)
