find_package(ament_cmake_catch2 REQUIRED)
find_package(Catch2 REQUIRED)

# Add test executable for the project
ament_add_catch2(${PROJECT_NAME}_test
    test_button_group_widget.cpp
    test_joystick_graphics_view_widget.cpp
    test_utils.cpp
)

# Link libraries to the test executable
target_link_libraries(${PROJECT_NAME}_test
    Catch2::Catch2WithMain
    Qt5::Test
    ${PROJECT_NAME}
)

# Specify include directories for the test target
target_include_directories(${PROJECT_NAME}_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
)
