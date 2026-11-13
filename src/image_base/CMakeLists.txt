add_executable(
    image_base
    main.cpp
)

target_link_libraries(
    image_base
    PRIVATE
    ${OpenCV_LIBS}   
)

set_target_properties(
    image_base
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    "${PROJECT_SOURCE_DIR}/target"
)