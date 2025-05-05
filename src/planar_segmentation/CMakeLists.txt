cmake_minimum_required(VERSION 3.8)
project(planar_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetReferencePoint.srv"
  # No DEPENDENCIES needed for primitives
)

# Define executable
add_executable(planar src/planar-segmentation.cpp)

# Add target dependencies
ament_target_dependencies(planar
  rclcpp
  rosidl_default_runtime # Still need this for runtime setup
  geometry_msgs
  std_msgs
  pcl_conversions
)

# Manually add the include directory for generated C++ headers
target_include_directories(planar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>)

# Explicitly Link generated type support library (using plain signature)
target_link_libraries(planar  # <-- REMOVED PUBLIC keyword
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# --- Keep the rest (testpcl target, install, testing, ament_package) ---

# Executable 'testpcl' (If you still need it)
add_executable(testpcl src/testpcl.cpp)
ament_target_dependencies(testpcl
  rclcpp
  std_msgs
  pcl_conversions
)

# Install targets
install(TARGETS
  planar
  testpcl
  DESTINATION lib/${PROJECT_NAME})

# Testing section
# ... (keep as is) ...

# Declare the package to ament
ament_package()