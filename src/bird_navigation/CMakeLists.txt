cmake_minimum_required(VERSION 3.8)
project(bird_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(g2si_cpp_node src/g2si_node.cpp)
ament_target_dependencies(g2si_cpp_node rclcpp sensor_msgs)
add_executable(pcl_crop_node src/pcl_crop_node.cpp)
ament_target_dependencies(pcl_crop_node rclcpp sensor_msgs pcl_ros tf2_ros)

# Install Cpp executables
install(TARGETS
  g2si_cpp_node
  pcl_crop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/g2si_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME g2si_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
