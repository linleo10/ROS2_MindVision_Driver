# cmake_minimum_required(VERSION 3.8)
# project(cam)

# set(cam_MAJOR_VERSION 0)
# set(cam_MINOR_VERSION 0)
# set(cam_PATCH_VERSION 0)
# set(cam_VERSION
#   ${cam_MAJOR_VERSION}.${cam_MINOR_VERSION}.${cam_PATCH_VERSION})

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)


cmake_minimum_required(VERSION 3.5)
project(cam)

# Find packages and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

# Add include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(mvs_node src/mvs_node.cpp)

# Link libraries
target_link_libraries(mvs_node
  ${OpenCV_LIBRARIES}
  CameraApi
)

# Install executable
install(TARGETS mvs_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  cv_bridge
  sensor_msgs
  image_transport
)

# Package export
ament_package()
