# leap_desc
This package contains the meshes and URDFs that describe the physical configuration of the Amiga.

## Notes
As always, all the source files should have headers at the top explaining what they do and individual notes.

This package isn't really intended to be standalone. I mean there's no point publishing a `/tf` tree if nothing is using it. I think it's currently being launched from `amiga_sensors.launch.py` in `leap_launch`.

A brief description of `amiga_base.xacro`: `base_link` is the overall describer of the Amiga. I placed it such that it aligns with the center of rotation, but it might not be perfect. `base_footprint` is the same thing except projected to the ground (I think it's used by both `leap_nav` and `leap_icp`). Individual transforms for each of the sensors are also present in this xacro. This xacro is the one used by everything on the physical robot, but the simulator requires further details. That's why this one is called "base" and the one in `leap_sim` extends it.