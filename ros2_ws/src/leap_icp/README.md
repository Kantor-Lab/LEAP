# leap_icp
This package uses fast_gicp's VGICP to localize the robot by matching the current LIDAR pointcloud against a prebuilt complete map.

## Notes
This package is really part of the `leap_control` ecosystem of localization. However, that package is python and this needs to be c++, so I made it a separate package. 

The source code for the actual ICP is in `thirdparty/fast_gicp`. When building that package, use `colcon build --symlink-install --packages-select fast_gicp --cmake-args -DBUILD_VGICP_CUDA=ON`.

