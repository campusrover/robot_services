# ROS robot service bridges

a custom package for sending certain ROS information to any other app through REDIS

## REDIS channels

1. "Map": Map JSON is `RPUSH`ed to this topic by `map_bridge.py`. Map data is sent as the dimentions of the map in `width` and `height` (in meters), a `line_count` representing the number of line segments in the JSON, and `data`, a list of line segments, represented as four-tuples representing the two endpoints of each line, [x1, y1, x2, y2]. line segment coordinates are in meters relative to the center of odometry if a TF from odom to map is available, otherwise they are relative to map center.
2. "Odom": Odom JSON is `SET` by `movement_bridge.py`. JSON includes `location` as [x, y, z], `orientation` as [roll, pitch, yaw], `linearvelocity` in m/s, `angularvelocity` in rad/s. robot location is in meters relative to the center of odometry. Orientation is in radians.
3. "cmd": read by `movement_bridge.py` to get movement commands from the other simulation

## Launch

`bridge.launch` is the primary launch file for this package. Presently, it launches the map, odometry, and movement command bridge nodes alongside SLAM. for the map bridge to work best, it needs a transform from `odom` to `map` to exist, which is provided by SLAM and AMCL. `bridge.launch` also has the following launch arguments:

* `mode` (with accepted values `patrol` and `command`, with `command` as the default). This argument changes which node is used to dictate robot movement - patrol autonomously or user inputted commands
* `odom_send_thresh` (float). This arg is used to change the rate at which odom updates are `set` to Redis. What the threshold describes is a sum in in the change in x position, y position and z orientation. A new update will only be sent once the total change since the past send has exceeded the threshold. Suggested values for this are between 0.1 and 0.5.

Use `roslaunch robot_services test.launch` to test the map bridge on any map. A few samples are included in this repo and the performance is logged in test.launch. the python module Pillow is required for test.launch.
