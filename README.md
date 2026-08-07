# LEAP

This repository contains all the code that I (Atharv Goel) wrote during the 2026 summer as part of the LEAP project. If you have questions, feel free to reach out to me on the Kantor Lab slack or at atharvgoel@cmu.edu. Each ROS 2 package should contain further notes beyond what I explain here, and many of the source files have headers at the top for explanations as well.

I basically accomplished the localization of the Amiga and some of the autonomous navigation. It was all predomoninantly tested within the highbay and the open area outside the garade door. The localization holds up well, but I think there's a lot of work to be done regarding the navigation portion. Check out `leap_nav` for some details on that.

## Building and Running

To run my code, you simply have to run the `launch.py` file:
>$ cd LEAP && python launch.py

I usually `ssh -Y` into the Amiga, so I can run it from my laptop.

When you run the launch file, a timestamped log file will be created in the `/logs` directory, which will contain the output of everything running. The record option in the launch file will create a rosbag if you want.

To build any packages, I usually just do `colcon build --symlink-install --packages-select <your package>` inside `/ros2_ws`. Notably, for `fast_gicp` specifically, do `colcon build --symlink-install --packages-select fast_gicp --cmake-args -DBUILD_VGICP_CUDA=ON` so that it builds with CUDA enabled. The `ouster-ros` package might also act difficult because it's linking so many things that the Jetson Nano runs out of RAM. If you ever need to rebuild it, you'll have to temporarily allocate more swap (8gb should be good).

## Todo
1. Fix the Amiga vs /cmd_vel controller discrepancy. Basically, the Amiga's low level controller handles the /cmd_vel messages, but it seems like it only uses wheel encoders. Since our Amiga is modified to remove the back two wheels, our center of rotation is right along the front set of wheels, but the Amiga settings don't let you account for that. Thus, the Amiga doesn't rotate at the correct speed. I have a bad patchjob by just scaling the /cmd_vel messages by 1.2 before sending it over CAN, but it doesn't seem great. I've emailed Bonsai asking for lower level control, as it would also be nice to be able to read the wheel encoders for our own odometry.
2. Edit my GUI / make a new GUI for queuing different nav2 commands. [Here](https://docs.google.com/document/d/1su4jOH8YGUaO75CbV_-c1mCwgwChBIqG8OsTZ_R6K4A/edit?usp=sharing) is a doc with more of my thoughts on that.
3. A "traversability" measurement of some sort to be used in the nav2 local costmap. I think just using some kind of jank voxel grid solution is a little too primitive. There's also the fact that its okay to hit some leaves overhead, but probably not branches. 
4. A custom planner (I left some details regarding this in the README in `leap_nav`).

#1 and #2 are probably not *too* bad but #3 and #4 will require a lot more work for sure.