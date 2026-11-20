cmake_minimum_required(VERSION 3.16)
project(dnn_test LANGUAGES CXX)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc video videoio highgui dnn)

add_executable(dnn_test main.cpp)
target_compile_features(dnn_test PRIVATE cxx_std_17)
target_include_directories(dnn_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../include" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dnn_test PRIVATE ${OpenCV_LIBS})
