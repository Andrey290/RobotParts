cmake_minimum_required(VERSION 3.8)
project(lds01_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find pigpiod_if2 library instead of pigpio
find_library(pigpiod_if2_LIBRARY pigpiod_if2)
if(NOT pigpiod_if2_LIBRARY)
  message(WARNING "pigpiod_if2 library not found - GPIO functionality will be disabled")
  set(BUILD_WITH_GPIO OFF)
else()
  set(BUILD_WITH_GPIO ON)
endif()

# Add C library first
add_library(parser_lib
  src/parser.c
  src/serial.c
  src/gpio.c
)

# Then specify include directories
target_include_directories(parser_lib PUBLIC
  include
)

# Then link libraries
if(BUILD_WITH_GPIO)
  target_link_libraries(parser_lib
    ${pigpiod_if2_LIBRARY}
    m
  )
  target_compile_definitions(parser_lib PUBLIC -DUSE_PIGPIOD_IF2)
else()
  target_link_libraries(parser_lib
    m
  )
  target_compile_definitions(parser_lib PUBLIC -DNO_GPIO)
endif()

# Add ROS node
add_executable(lidar_node src/lidar_node.cpp)

target_include_directories(lidar_node PUBLIC
  include
)

ament_target_dependencies(lidar_node
  rclcpp
  sensor_msgs
)

target_link_libraries(lidar_node
  parser_lib
)

# Install targets
install(TARGETS
  lidar_node
  parser_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(parser_lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
