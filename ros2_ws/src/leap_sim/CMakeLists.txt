cmake_minimum_required(VERSION 3.8)
project(leap_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# IMU Covariance Injector
add_executable(imu_covariance_injector
  src/imu_covariance_injector.cpp
)
target_include_directories(imu_covariance_injector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(imu_covariance_injector PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(imu_covariance_injector
  rclcpp
  sensor_msgs
)

# GPS Covariance Injector
add_executable(gps_covariance_injector
  src/gps_covariance_injector.cpp
)
target_include_directories(gps_covariance_injector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(gps_covariance_injector PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(gps_covariance_injector
  rclcpp
  sensor_msgs
)

# Install executables
install(
  TARGETS
    imu_covariance_injector
    gps_covariance_injector
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY 
    launch
    config
    models
    urdf
    worlds
    rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
