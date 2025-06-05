cmake_minimum_required(VERSION 3.12)
project(ros_virtual_joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui Test)
find_package(rclcpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

################
# Core library #
################
add_library(${PROJECT_NAME} SHARED
  include/ros_virtual_joystick/widget.hpp
  src/publisher.cpp
  src/utils.cpp
  src/widget.cpp
  src/widgets/button_group_widget.cpp
  src/widgets/joystick_graphics_view_widget.cpp
  src/widgets/joystick_widget.cpp
  src/widgets/topic_widget.cpp
  src/widgets/widget_impl.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

###################
# Main Executable #
###################
add_executable(${PROJECT_NAME}_node
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

##################
# RQt GUI Plugin #
##################
add_library(${PROJECT_NAME}_rqt_plugin SHARED
  src/plugins/rqt_plugin.cpp
)

ament_target_dependencies(${PROJECT_NAME}_rqt_plugin
  pluginlib
  rclcpp
  rqt_gui_cpp
)

target_link_libraries(${PROJECT_NAME}_rqt_plugin
  ${PROJECT_NAME}
)

##################
# RViz GUI Plugin #
##################
add_library(${PROJECT_NAME}_rviz_plugin SHARED
  src/plugins/rviz_plugin.cpp
)

ament_target_dependencies(${PROJECT_NAME}_rviz_plugin
  pluginlib
  rclcpp
  rviz_common
)

target_link_libraries(${PROJECT_NAME}_rviz_plugin
  ${PROJECT_NAME}
)

###########
# Testing #
###########
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

###########
# Install #
###########
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  plugin_rqt.xml
  plugin_rviz.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin_rqt.xml")
pluginlib_export_plugin_description_file(rviz_common "plugin_rviz.xml")

# Install & Export
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_rqt_plugin
    ${PROJECT_NAME}_rviz_plugin
  EXPORT export_${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  pluginlib
  rclcpp
  rqt_gui_cpp
  rviz_common
  sensor_msgs
)
ament_export_targets(export_${PROJECT_NAME}_component HAS_LIBRARY_TARGET)

ament_package()
