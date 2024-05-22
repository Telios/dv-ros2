cmake_minimum_required(VERSION 3.8)
project(dv_ros2_imu_bias)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dv_ros2_msgs REQUIRED)
find_package(dv_ros2_messaging REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies "rclcpp" "dv_ros2_msgs" "dv_ros2_messaging" "std_msgs" "sensor_msgs" "geometry_msgs")

add_executable(${PROJECT_NAME}_node src/main.cpp)

ADD_SUBDIRECTORY(../dv-processing ../dv-processing EXCLUDE_FROM_ALL)

ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

target_link_libraries(${PROJECT_NAME}_node dv::processing)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_node
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
