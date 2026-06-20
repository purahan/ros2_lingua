cmake_minimum_required(VERSION 3.8)
project(ros2_lingua_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_lingua_interfaces REQUIRED)

# --- Header-only library (interface target) ---
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp
  std_msgs
  ros2_lingua_interfaces
)

# --- Install headers ---
install(
  DIRECTORY include/
  DESTINATION include
)

# --- Export for downstream packages ---
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  ros2_lingua_interfaces
)

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_capability test/test_capability.cpp)
  target_include_directories(test_capability PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_capability
    rclcpp
    std_msgs
    ros2_lingua_interfaces
  )

  ament_add_gtest(test_compilation test/test_compilation.cpp)
  target_include_directories(test_compilation PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_compilation
    rclcpp
    std_msgs
    ros2_lingua_interfaces
  )
endif()

ament_package()
