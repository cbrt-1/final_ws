cmake_minimum_required(VERSION 3.8)
project(planar_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED) # This package brings in PCL dependency

# Executable 'planar'
add_executable(planar src/planar-segmentation.cpp)
ament_target_dependencies(planar
  rclcpp
  std_msgs
  pcl_conversions # This handles includes and linking for ROS pkgs AND PCL
)

# Executable 'testpcl'
add_executable(testpcl src/testpcl.cpp)
# Add ALL dependencies needed by testpcl here using ament_target_dependencies
ament_target_dependencies(testpcl
  rclcpp          # Add if testpcl uses ROS features
  std_msgs        # Add if testpcl uses standard messages
  pcl_conversions # This handles includes and linking for PCL
)

# Remove these manual PCL lines - let ament_target_dependencies handle it
# target_include_directories(testpcl PUBLIC ${PCL_INCLUDE_DIRS})
# target_link_libraries(testpcl PUBLIC ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  planar
  testpcl
  DESTINATION lib/${PROJECT_NAME})

# Testing section (looks okay, adjust linters as needed)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # comment the line below when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # comment the line below when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare the package to ament
ament_package()