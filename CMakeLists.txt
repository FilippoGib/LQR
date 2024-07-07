cmake_minimum_required(VERSION 3.5)
project(lqr)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mmr_base REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mmr_common_functions REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL 1.13 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(lqr_node src/lqr_node.cpp src/frenetSpace.cpp)

# Link against PCL libraries and other dependencies
target_link_libraries(lqr_node 
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(lqr_node
  rclcpp
  std_msgs
  mmr_base
  nav_msgs
  mmr_common_functions
  ackermann_msgs
  geometry_msgs
  tf2_ros
  tf2_kdl
  tf2
  tf2_geometry_msgs
  PCL
  Eigen3
  sensor_msgs
  rclcpp_components
)

install(TARGETS
  lqr_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY
      config
      DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
	    launch
	    DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
