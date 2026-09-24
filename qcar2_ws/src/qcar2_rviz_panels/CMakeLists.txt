cmake_minimum_required(VERSION 3.8)
project(qcar2_rviz_panels)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/qcar_control_panel.cpp
  src/qcar_control_panel.hpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
)
ament_target_dependencies(${PROJECT_NAME} rviz_common rviz_rendering pluginlib rclcpp std_msgs sensor_msgs)
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_NO_KEYWORDS)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES plugins_description.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
ament_export_libraries(${PROJECT_NAME})
ament_package()
