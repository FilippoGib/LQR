cmake_minimum_required(VERSION 3.5)
project(lqr)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL 1.13 REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(lqr_node src/LQR_node.cpp src/frenetSpace.cpp)
ament_target_dependencies(lqr_node rclcpp std_msgs geometry_msgs)

target_link_libraries(lqr_node ${PCL_LIBRARIES})

install(TARGETS
  lqr_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()
