# leap_sensors
This package contains specific config files and launch files for the individual sensor.

## Notes
Basically all taken and cleand up from Billy's code, I dont really know a bunch of specifics.

I *do* know that `timestamp_mode` in `ouster_params.yaml` has been a source of some issue. In orderfor the LIDAR data to be used for localization, it needs ot be timestamped such that it matches with everything else, hence `'TIME_FROM_ROS_TIME'`. HOWEVER, apparently this isn't as accurate as the built in `'TIME_FROM_INTERNAL_OSC'`. In fact, Billy says that when generating the map, the using ROS timestamps isntead of the builtin timer is much worse.