cmake_minimum_required(VERSION 3.8)
project(bird_drivetrain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(serial REQUIRED)

include_directories(include)

add_executable(clearcore_driver_node src/clearcore_driver_node.cpp)
ament_target_dependencies(clearcore_driver_node rclcpp geometry_msgs std_msgs std_srvs serial)
add_executable(linac_driver_node src/linac_driver_node.cpp)
ament_target_dependencies(linac_driver_node rclcpp geometry_msgs std_msgs std_srvs serial)

install(TARGETS
  linac_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  clearcore_driver_node
  DESTINATION lib/${PROJECT_NAME}
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
