# leap_nav
This package contains the navigation stuff for the robot (mostly nav2 stuff).

## Notes
There's definitely a lot of room for improvement in this package. A custom planner would probably be better than the SMAC planner right now, and the controller could be tuned. It's hard for me to exactly say because the Amiga's blackbox controller is definitely causing issues too. However, it does feel like the planner makes unnecessary curves sometimes, and is overly aggressive other times. It uses a cost map, but this does mean that's it's willing to get closer to danger than it should. Considering the intended environment, it should moreso always be picking a safe path, straight and down the middle, and only venture away from that if it absolutely has to. I've not actually tested in a tree nursery, so take my words with a grain of salt. I'm sure a cramped environemnt also some kind of terrian viability function will have drastic effects.

The SMAC planner and MPPI controller aren't there for any specific reasons other than the fact that I got them working best.

Local costmap definitely has some problems. The issue is that I'm not recording the pitch or roll of the robot at all, so if the Amiga is on a slope, it interprets the ground as an obstacle. I tried using the voxel thing that's there now but it's still kinda sus. 