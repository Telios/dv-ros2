cmake_minimum_required(VERSION 3.8)
project(dv_ros2_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dv_ros2_msgs REQUIRED)
find_package(dv_ros2_messaging REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(dv-processing REQUIRED)

set(dependencies "std_msgs" "rclcpp" "geometry_msgs" "sensor_msgs" "dv_ros2_msgs" "dv_ros2_messaging" "tf2_msgs")

include_directories(include())

file(GLOB_RECURSE SOURCES src/*.cpp)

add_library(${PROJECT_NAME}_core
  ${SOURCES}
  )

add_executable(${PROJECT_NAME}_node
  src/capture_node.cpp
  src/Capture.cpp
  )

ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES} dv::processing)

target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  dv::processing
  )

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  )

install(TARGETS
  ${PROJECT_NAME}_node
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
