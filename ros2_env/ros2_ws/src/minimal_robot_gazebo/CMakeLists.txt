cmake_minimum_required(VERSION 3.8)
project(minimal_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_ign REQUIRED)  # This finds ros-ign packages for Ignition Gazebo
find_package(ros_ign_gazebo REQUIRED)  # This is the ros-ign gazebo integration

# Install the launch folder
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the URDF folder
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install the worlds folder
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()