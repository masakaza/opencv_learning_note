# 创建可执行文件
add_executable(test
    main.cpp
)

# 链接 OpenCV 库
target_link_libraries(test PRIVATE
    ${OpenCV_LIBS}
)

# 设置可执行文件输出目录
set_target_properties(test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/target"
)