add_executable(
    cap_video
    main.cpp
)

target_link_libraries(
    cap_video
    PRIVATE
    ${OpenCV_LIBS}   
)

set_target_properties(
    cap_video
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    "${PROJECT_SOURCE_DIR}/target"
)