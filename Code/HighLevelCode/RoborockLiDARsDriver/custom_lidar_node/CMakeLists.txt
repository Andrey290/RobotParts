cmake_minimum_required(VERSION 3.16)
project(custom_lidar_node)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Для отладки, можно вывести найденные пути
message(STATUS "rclcpp_INCLUDE_DIRS: ${rclcpp_INCLUDE_DIRS}")
message(STATUS "rclcpp_LIBRARIES: ${rclcpp_LIBRARIES}")

# Глобальные флаги компилятора
add_compile_options(-Wall -Wextra -Wpedantic)

# заглушку для GPIO
add_definitions(-DNO_PIGPIO)

# Лидар нода (использует GPIO заглушку)
add_executable(lidar_node
  src/lidar_node.cpp
  src/serial.c
  src/parser.c
  src/gpio_dummy.c  # Используем заглушку вместо gpio.c
)

# Лидар нода
# add_executable(lidar_node
#   src/lidar_node.cpp
#   src/serial.c
#   src/parser.c
#   src/gpio.c
# )

# # Для C файлов отключаем C++ флаги
set_source_files_properties(
  src/serial.c
  src/parser.c
  src/gpio_dummy.c
  PROPERTIES LANGUAGE C
)

# # Целевые директории include
target_include_directories(lidar_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

# target_include_directories(lidar_node PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )


# Целевые библиотеки
target_link_libraries(lidar_node
  rclcpp::rclcpp
  ${sensor_msgs_LIBRARIES}
  m
)

# target_link_libraries(lidar_node
#   ${rclcpp_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
#   # ${PIGPIO_LIB}
#   m
# )

add_executable(lidar_simulator
  src/lidar_simulator.cpp
)

target_include_directories(lidar_simulator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

# target_include_directories(lidar_node PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

target_link_libraries(lidar_simulator
  rclcpp::rclcpp
  ${sensor_msgs_LIBRARIES}
)

# target_link_libraries(lidar_simulator
#   ${rclcpp_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
# )

# Управляющая нода
add_executable(control_node
  src/control_node.cpp
  src/stm32_interface.cpp
  src/movement.cpp
)

target_include_directories(control_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)


# target_include_directories(control_node PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

target_link_libraries(control_node
  rclcpp::rclcpp
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# target_link_libraries(control_node
#   ${rclcpp_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
#   ${geometry_msgs_LIBRARIES}
#   ${std_msgs_LIBRARIES}
# )

# Установка
install(TARGETS lidar_node lidar_simulator control_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()

