# 指定最低 CMake 版本
cmake_minimum_required(VERSION 3.16)

# 创建项目
project(OpenCV_Project)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置 OpenCV 4 配置文件所在目录
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4" CACHE PATH "OpenCV 4 config directory" FORCE)
include_directories(${PROJECT_SOURCE_DIR}/include)


# 查找 OpenCV
find_package(OpenCV REQUIRED CONFIG)

# 输出 OpenCV 信息
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV Include: ${OpenCV_INCLUDE_DIRS}")

# 添加各个子项目
# add_subdirectory(src/image_read image_read)
# add_subdirectory(src/image_process image_process)
# add_subdirectory(src/camera camera)

add_subdirectory(src/test test)
add_subdirectory(src/image_base image_base)
add_subdirectory(src/task_01 task_01)
add_subdirectory(src/task_02 task_02)
add_subdirectory(src/task_03 task_03)
add_subdirectory(src/task_04 task_04)
add_subdirectory(src/task_05 task_05)
add_subdirectory(src/task_06 task_06)
add_subdirectory(src/task_07 task_07)
add_subdirectory(src/cap_video cap_video)