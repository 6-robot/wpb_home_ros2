cmake_minimum_required(VERSION 3.8)
project(wpb_home_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# executeable
add_executable(wpb_home_core 
  src/wpb_home_core.cpp
  src/driver/SerialCom.cpp
  src/driver/WPB_Home_driver.cpp
  )
ament_target_dependencies(wpb_home_core 
"rclcpp" "std_msgs" "geometry_msgs" "nav_msgs" "tf2" "sensor_msgs" "tf2_ros")

add_executable(wpb_home_js_vel 
  src/wpb_home_js_vel.cpp
  )
ament_target_dependencies(wpb_home_js_vel 
"rclcpp" "std_msgs" "sensor_msgs" "geometry_msgs" )

add_executable(wpb_home_calibrate_velocity 
  src/wpb_home_calibrate_velocity.cpp
  )
ament_target_dependencies(wpb_home_calibrate_velocity 
"rclcpp" "std_msgs" "geometry_msgs" )

add_executable(wpb_home_lidar_filter 
  src/wpb_home_lidar_filter.cpp
  )
ament_target_dependencies(wpb_home_lidar_filter 
"rclcpp" "std_msgs" "sensor_msgs" )

add_executable(wpb_home_motor_encoder 
  src/wpb_home_motor_encoder.cpp
  src/driver/SerialCom.cpp
  src/driver/WPB_Home_driver.cpp
  )
ament_target_dependencies(wpb_home_motor_encoder 
"rclcpp" "geometry_msgs" )

add_executable(wpb_home_test_motors 
  src/wpb_home_test_motors.cpp
  src/driver/SerialCom.cpp
  src/driver/WPB_Home_driver.cpp
  )
ament_target_dependencies(wpb_home_test_motors 
"rclcpp" )

add_executable(wpb_home_test_mani 
  src/wpb_home_test_mani.cpp
  src/driver/SerialCom.cpp
  src/driver/WPB_Home_driver.cpp
  )
ament_target_dependencies(wpb_home_test_mani 
"rclcpp" )

# install
install(TARGETS
  wpb_home_core
  wpb_home_js_vel
  wpb_home_calibrate_velocity
  wpb_home_lidar_filter
  wpb_home_motor_encoder
  wpb_home_test_motors
  wpb_home_test_mani
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY
  rviz config launch
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
