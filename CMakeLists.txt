cmake_minimum_required(VERSION 3.8)
project(alliedvision)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND CMAKE_PREFIX_PATH "/opt/VimbaX_2023-4/api")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(auv_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Vmb REQUIRED COMPONENTS CPP NAMES Vmb VmbC VmbCPP VmbImageTransform)

add_executable(alliedvision_control src/alliedvision_control.cpp)
ament_target_dependencies(alliedvision_control PUBLIC
  "OpenCV"
  auv_interfaces
  cv_bridge
  rclcpp
  sensor_msgs
  Vmb
)
target_link_libraries(alliedvision_control PRIVATE
  ${OpenCV_LIBRARIES}
  Vmb::CPP
)
target_include_directories(alliedvision_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  alliedvision_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
