# leap_desc

This package is the single source of truth for the physical configuration of the LEAP robot. It contains the URDF/Xacro descriptions, 3D meshes, and the Robot State Publisher launch files required to broadcast the robot's coordinate frames (`/tf`) for both simulation and physical hardware deployments. It is not intended to be used standalone, and should be referenced by other ROS packages only.

## Directory Structure

```text
leap_sim/
├── CMakeLists.txt
├── package.xml
├── launch/
├── urdf/
├── meshes/
```

### Subdirectory Breakdown

*   **`launch/`**
    Contains the launch files.
*   **`urdf/`**
    Contains the descriptions of the robot. Xacro files are directly used now, but some corresponding urdf files may also exist (albeit out of date).
*   **`meshes/`**
    Contains the 3D visual and collision meshes,  for the robots. It it is primarily referred to by files in the `urdf\` directory.

## Build Instructions

Ensure you are in the root of your ROS 2 workspace, then build the package:

```bash
colcon build --packages-select leap_desc
source install/setup.bash
```

## Developer Notes
*   The keen observer might notice a `config\` folder. This folder only contains some controller related things that the robot xacro file references. It should eventually be moved to a different leap_control package or something of the like.