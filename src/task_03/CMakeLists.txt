# 创建可执行文件
add_executable(task_03
    main.cpp
)

# 链接 OpenCV 库
target_link_libraries(task_03 PRIVATE
    ${OpenCV_LIBS}
)

# 设置可执行文件输出目录
set_target_properties(task_03 PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/target"
)