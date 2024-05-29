cmake_minimum_required(VERSION 3.8)
project(dv_ros2_runtime_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dv_ros2_msgs REQUIRED)
find_package(dv_ros2_messaging REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dv REQUIRED)

set(dependencies "std_msgs" "rclcpp" "image_transport" "dv_ros2_msgs" "dv_ros2_messaging" "sensor_msgs")

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/dv
    /usr/${CMAKE_INSTALL_DATAROOTDIR}/dv
    ${CMAKE_INSTALL_PREFIX}/share/dv
    /usr/share/dv
    /usr/local/${CMAKE_INSTALL_DATAROOTDIR}/dv
    /usr/local/share/dv
    )

set(DV_LIBRARIES dv::sdk)

include_directories(include() ${rclcpp_INCLUDE_DIRS} ${DV_INCLUDE_DIRS})

set(CMAKE_INSTALL_RPATH ${DV_MODULES_DIR})

add_library(ImageBridge SHARED src/Ros2ImageBridge.cpp)
ament_target_dependencies(ImageBridge ${rclcpp_EXPORTED_TARGETS} ${dependencies})

set_target_properties(ImageBridge PROPERTIES PREFIX "ros2_")
target_link_libraries(ImageBridge  ${DV_LIBRARIES} ${rclcpp_LIBRARIES})

add_library(ImuBridge SHARED src/Ros2IMUBridge.cpp)
ament_target_dependencies(ImuBridge ${rclcpp_EXPORTED_TARGETS} ${dependencies})

set_target_properties(ImuBridge PROPERTIES PREFIX "ros2_")

target_link_libraries(ImuBridge  ${DV_LIBRARIES} ${rclcpp_LIBRARIES})

add_library(EventBridge SHARED src/Ros2EventBridge.cpp include/dv_ros2_runtime_modules/Ros2EventBridge.hpp)
ament_target_dependencies(EventBridge ${rclcpp_EXPORTED_TARGETS} ${dependencies})

set_target_properties(EventBridge PROPERTIES PREFIX "ros2_")

target_link_libraries(EventBridge  ${DV_LIBRARIES} ${rclcpp_LIBRARIES})

add_library(TriggerBridge SHARED src/Ros2TriggerBridge.cpp)
ament_target_dependencies(TriggerBridge ${rclcpp_EXPORTED_TARGETS} ${dependencies})

set_target_properties(TriggerBridge PROPERTIES PREFIX "ros2_")

target_link_libraries(TriggerBridge  ${DV_LIBRARIES} ${rclcpp_LIBRARIES})

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
