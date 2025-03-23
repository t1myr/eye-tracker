cmake_minimum_required(VERSION 3.20)
project(eye_tracker_test)

enable_testing()

find_package(GTest CONFIG REQUIRED)


file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_executable(eye_tracker_tests ${SOURCE_FILES} main.cpp)
target_link_libraries(eye_tracker_tests PRIVATE eye_tracker_lib GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
