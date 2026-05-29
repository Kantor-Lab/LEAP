cmake_minimum_required(VERSION 3.8)
project(leap_icp)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_executable(map_odom_localizer src/map_odom_localizer.cpp)

target_include_directories(map_odom_localizer PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_definitions(map_odom_localizer PRIVATE
    THRUST_IGNORE_CUB_VERSION_CHECK
    EIGEN_NO_CUDA
)

ament_target_dependencies(map_odom_localizer
  rclcpp
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  fast_gicp
)

# Install rules
install(TARGETS map_odom_localizer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    maps
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
