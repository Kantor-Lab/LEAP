# leap_sim

This package contains the Gazebo simulation environment for the LEAP project. It provides the worlds, environmental models, and launch configurations required to simulate the robot's operating environment (e.g., the plant nursery/orchard) for testing autonomy and navigation stacks.

## Directory Structure

```text
leap_sim/
├── CMakeLists.txt
├── package.xml
├── launch/
├── urdf/
├── meshes/
├── models/
└── worlds/
```

### Subdirectory Breakdown

*   **`launch/`**
    Contains the launch files. `gazebo.launch.py` is the main entry point.
*   **`urdf/`**
    Contains the descriptions of the robot. Xacro files are directly used now, but some corresponding urdf files may also exist (albeit out of date).
*   **`meshes/`**
    Contains the 3D visual and collision meshes,  for the robots. It it is primarily referred to by files in the `urdf\` directory.
*   **`models/`**
    Contains the 3D visual and collision meshes, materials, and specific `.sdf` configurations for environmental props. This includes all the assets required to render the trees, ground planes, or obstacles used in the simulation.
*   **`worlds/`**
    Contains the layout files that define the actual simulation environments. This includes the static `.world` files loaded by Gazebo, as well as the `.sdf.erb` (Embedded Ruby) templates used to procedurally generate those layouts (like spacing out rows of trees).

## Build Instructions

Ensure you are in the root of your ROS 2 workspace, then build the package:

```bash
colcon build --packages-select leap_sim
source install/setup.bash
```

## Usage

To launch the standalone simulation world:

```bash
ros2 launch leap_sim gazebo_sim.launch.py
```

## Developer Notes

*   **Organization:** This package should only handle simulation-specific code. That means the `urdf\` and `meshes\` folder should eventually be moved out of this package.