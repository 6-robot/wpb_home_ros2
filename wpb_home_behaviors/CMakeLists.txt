cmake_minimum_required(VERSION 3.8)
project(wpb_home_behaviors)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
foreach(dep rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs tf2_ros
    pcl_conversions pcl_ros wpb_home_msgs)
  find_package(${dep} REQUIRED)
endforeach()
include_directories(include)
add_executable(wpb_home_objects_3d src/objects_3d.cpp)
ament_target_dependencies(wpb_home_objects_3d rclcpp std_msgs sensor_msgs geometry_msgs
  visualization_msgs tf2_ros pcl_conversions pcl_ros wpb_home_msgs)
add_executable(wpb_home_grab src/grab.cpp)
ament_target_dependencies(wpb_home_grab rclcpp std_msgs sensor_msgs geometry_msgs
  wpb_home_msgs)
install(TARGETS wpb_home_objects_3d wpb_home_grab DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs
  tf2_ros pcl_conversions pcl_ros wpb_home_msgs)
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_object_tracking.py")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(object_tracking test/test_object_tracking.py TIMEOUT 120
    ENV BEHAVIORS_BIN_DIR=${CMAKE_CURRENT_BINARY_DIR} ROS_LOCALHOST_ONLY=1)
endif()
ament_package()
