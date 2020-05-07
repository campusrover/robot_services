# ROS robot service bridges

a custom package for sending certain ROS information to any other app through REDIS

## REDIS channels

1. "Map": Map JSON is `RPUSH`ed to this topic by `map_bridge.py`. Map data is sent as the dimentions of the map in `width` and `height` (in meters), a `line_count` representing the number of line segments in the JSON, and `data`, a list of line segments, represented as four-tuples representing the two endpoints of each line, [x1, y1, x2, y2]. line segment coordinates are in meters relative to the center of odometry if a TF from odom to map is available, otherwise they are relative to map center.
2. "Odom": Odom JSON is `SET` by `movement_bridge.py`. JSON includes `location` as [x, y, z], `orientation` as [roll, pitch, yaw], `linearvelocity` in m/s, `angularvelocity` in rad/s. robot location is in meters relative to the center of odometry. Orientation is in radians.
3. "cmd": read by `movement_bridge.py` to get movement commands from the other simulation

## Launch

to launch, use `roslaunch robot_services bridge.launch`. By default this launches using the launch file command.launch. To use patrol.launch instead, use `roslaunch robot_services bridge.launch mode:=patrol`

Use `roslaunch robot_services test.launch` to test the map bridge on any map. A few samples are included in this repo and the performance is logged in test.launch. the python module Pillow is required for test.launch.
