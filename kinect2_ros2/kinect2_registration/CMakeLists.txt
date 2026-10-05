cmake_minimum_required(VERSION 3.8)
project(kinect2_registration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)

set(DEPTH_REG_CPU ON)
set(DEPTH_REG_OPENCL OFF)
message(STATUS "Depth registration: CPU only (OpenCL is not required)")
add_library(kinect2_registration SHARED
  src/kinect2_registration.cpp
  src/depth_registration_cpu.cpp)
target_compile_definitions(kinect2_registration PRIVATE DEPTH_REG_CPU)
target_include_directories(kinect2_registration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(kinect2_registration rclcpp std_msgs OpenCV)
target_link_libraries(kinect2_registration Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(kinect2_registration OpenMP::OpenMP_CXX)
endif()
install(TARGETS kinect2_registration
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/kinect2_registration DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(kinect2_registration)
ament_export_dependencies(rclcpp std_msgs OpenCV)
ament_package(CONFIG_EXTRAS cmake/kinect2_registration.cmake.in)
