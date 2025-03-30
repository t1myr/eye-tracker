cmake_minimum_required(VERSION 3.20)

project(cmake_eye_tracker VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(USE_AVX_INSTRUCTIONS ON CACHE BOOL "Use AVX instructions")

set(EYE_TRACKER_HEADERS_DIR ${PROJECT_SOURCE_DIR}/src/app)

include_directories(${EYE_TRACKER_HEADERS_DIR})


add_subdirectory(app)
add_subdirectory(test)

