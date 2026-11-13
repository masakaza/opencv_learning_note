cmake_minimum_required(VERSION 3.16)
project(hsv_range_visualizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

add_executable(hsv_range_visualizer main.cpp)
target_include_directories(hsv_range_visualizer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hsv_range_visualizer PRIVATE ${OpenCV_LIBS})
