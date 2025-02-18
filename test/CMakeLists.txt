cmake_minimum_required(VERSION 3.20)
project(eye_tracker_test)

add_subdirectory(lib/googletest)

include_directories(${EYE_TRACKER_HEADERS_DIR})
include_directories(lib/googletest/googletest/include)

file(GLOB_RECURSE SRC_FILES "src/*.cpp")

add_executable(eye_tracker_tests ${SOURCE_FILES} main.cpp)
target_link_libraries(eye_tracker_tests eye_tracker_lib gtest)
install(TARGETS eye_tracker_tests DESTINATION bin)