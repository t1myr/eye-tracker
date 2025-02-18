cmake_minimum_required(VERSION 3.20)

project(eye_tracker VERSION 1.0 LANGUAGES CXX)
project(eye_tracker_lib LANGUAGES CXX)

file(GLOB_RECURSE SRC_FILES "app/*.cpp")
file(GLOB_RECURSE HEADER_FILES "app/*.hpp")

add_subdirectory(lib/opencv)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(eye_tracker_lib SHARED STATIC ${HEADER_FILES} ${SRC_FILES})

set_target_properties(eye_tracker_lib PROPERTIES LINKER_LANGUAGE CXX)
install(TARGETS eye_tracker_lib DESTINATION ${EYE_TRACKER_INSTALL_LIB_DIR})
install(FILES ${HEADER_FILES} DESTINATION ${EYE_TRACKER_INSTALL_INCLUDE_DIR})

add_executable(eye_tracker ${SRC_FILES} main.cpp)
target_link_libraries(eye_tracker eye_tracker_lib opencv_core opencv_imgcodecs opencv_highgui)
install(TARGETS eye_tracker DESTINATION ${EYE_TRACKER_INSTALL_BIN_DIR})
