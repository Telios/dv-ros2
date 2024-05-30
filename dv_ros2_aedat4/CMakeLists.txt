cmake_minimum_required(VERSION 3.8)
project(dv_ros2_aedat4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dv_ros2_msgs REQUIRED)
find_package(dv_ros2_messaging REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dv-processing REQUIRED)

set(DEPENDENCIES "dv_ros2_msgs" "dv_ros2_messaging" "rosbag2_cpp" "std_msgs" "sensor_msgs" "rclcpp")
# Git submodule automatic update support.
find_package(Git QUIET)

if(Git_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../.git")
  # Update submodules as needed
  option(GIT_SUBMODULE "Check-out submodules during build" ON)

  if(GIT_SUBMODULE)
    message(STATUS "Git submodule update")

    execute_process(
      COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../
      RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
      message(
        FATAL_ERROR
          "${GIT_EXECUTABLE} submodule update --init failed with ${GIT_SUBMOD_RESULT}, please check-out submodules manually."
      )
    endif()
  endif()
endif()

add_subdirectory(thirdparty/CLI11)

add_executable(convert_aedat4 src/convert_aedat4.cpp)

ament_target_dependencies(convert_aedat4 ${DEPENDENCIES})

target_link_libraries(convert_aedat4 dv::processing CLI11::CLI11)

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
