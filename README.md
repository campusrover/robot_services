# ROS robot service bridges

a custom package for sending certain ROS information to any other app through REDIS. this package provides channels that are aimed to encourage merging robots with non robotic applictions. These channels will allow anyone wwith little or no programming experience, and no ROS experience, to be able to interact with robots for specific needs.

## Installation and setup

This package runs on Python 2.7

Besides default packages that are included with python and ros installations, the redis python module is required. Get it with `pip install redis`. To communicate with redis on your local device instead of a redis server, use `apt install redis-server`.

To use `test.launch` and `map_bridge_debug.py` you will need PIL. Get it with `pip install Pillow`. `map_bridge_debug.py` draws a map based on the line segments it generates to help visualize while debgging.

## REDIS channels

1. "Map": Map JSON is `RPUSH`ed to this topic by `map_bridge.py`. `LPOP` should be used to retrieve Map JSON from Redis. Map data is formatted as follows:
    * `width` and `height` (in meters) 
    * `line_count` representing the number of line segments in the JSON
    * `data`, a list of line segments, represented as four-tuples representing the two endpoints of each line, [x1, y1, x2, y2]. line segment coordinates are in meters relative to the center of odometry if a TF from odom to map is available, otherwise they are relative to map center defined by origin from the map topic.
2. "Odom": Odom JSON is `SET` by `movement_bridge.py`. JSON includes: 
    * `location` as [x, y, z]
    * `orientation` as [roll, pitch, yaw]
    * `linearvelocity` in m/s
    * `angularvelocity` in rad/s. robot location is in meters relative to the center of odometry. Orientation is in radians.
3. "Bridge_Reset": a `SET` value, should be either `0` or `1`. `1` indicates a request for a reset of all Redis keys. Keys will be reset with their normal JSON structure, with 0 values in each field. After key reset occurs, the value of this key will be `SET` to `0` by `reset_bridge.py`
4. "Cmd": read by `cmdListener.py`
5. "Log": Strings `RPUSH`ed to this key. Every string is a roslog message published by any node. each string includes the log type, where it came from, and the message.
6. "Fiducials: JSON that is `RPUSH`ed to a list structure. JSON includes info like:
    * `fid_count` representing the number of known fiducials
    * `dict` representing the fiducial marker format (see aruco marker documentation) 
    * `frame`, which will be `odom` if the transform from the camera link to odom is available, otherwise it will be `camera`, indicating that all values in the fiducial list are in coordinates relative to the robot, not the center of odometry. 
    * `data` is a list of known fiducial markers, where each element has a `fid` representing the marker's id number, and a `pose` which has `location` and `orientation` components (in euler radians)

## Namespacing

To prevent collisions of multiple users on the same redis server, this package provides namespacing. Set the rosparam `redis_ns` to your namespace, and as a result, any of the channels above will be prefixed with your namespace. For example, a user with the namespace "greg" will have all of their redis channels prefixed with "greg/", e.g. "greg/Map", "greg/Odom" . . .

## Launch

`bridge.launch` is the primary launch file for this package. Presently, it launches the map, odometry, reset, and movement command bridge nodes alongside SLAM. for the map bridge to work best, it needs a transform from `odom` to `map` to exist, which is provided by SLAM and AMCL (SLAM was chosen in this instance). `bridge.launch` also has the following launch arguments:

* `mode` (with accepted values `patrol` and `command`, with `command` as the default). This argument changes which node is used to dictate robot movement - patrol autonomously or user inputted commands
* `odom_send_thresh` (float). This arg is used to change the rate at which odom updates are `set` to Redis. What the threshold describes is a sum in in the change in x position, y position and z orientation. A new update will only be sent once the total change since the past send has exceeded the threshold. Suggested values for this are between 0.1 and 0.5. The default value is 0, meaning odom updates are sent each time they are received.

Use `roslaunch robot_services test.launch` to test the map bridge on any map. A few samples are included in this repo and the performance is logged in test.launch. the python module Pillow is required for test.launch.

## Reset

Every new bridge node should support the reset operation. A reset can be requested by setting the key `Bridge_Reset` to `1`. On the ROS side, this proliferates a reset command via the `/reset` topic using a `std_msgs/Empty` message type. All each node has to do is subscribe to `/reset` and if they receive a message, their callback should reset all of the node's respective keys. Resets will set the value of each key to contain a JSON with that key's expected structure, but 0 in each field (for list keys, there will be just one 0-populated JSON)

## Movement Commands

General commands supported:

* go forward (X) - moves the robot to a location x (or default of 1.0) meters ahead of where it currently is.
* go to X Y - moves the robot to the given (x,y) coordinate on its map.
* turn left (X) - turns the robot x (or default of 90) degrees to the left.
* turn right (X) - turns the robot x (or default of 90) degrees to the right.
* patrol - tells the robot to explore its environment.

For more detailed documentation on commands, controls, and feedback please read this documentation on [movement and feedback](robot_movement_and_feedback.txt)

### Loose Ends and Areas for Improvement

For an overview of the strategy and algorithms used in the map bridge, please read [this markdown file](map_bridge.md)

TODO list of future features:

* generic_bridge.py was initially created with hopes of creating a generic subsccriber-publisher from ros to redis and vice-versa that would perhaps make use of rospy.msg.AnyMsg.
* consolodate_lines in map_bridge tends to chug and take a lot of time when faced with many lines. additionally, the process of making a numpy array of an occupancy grid can be very slow if the occupancy grid is large (see basement_map_gen4 in cr_ros_3). Speeding up these operations would be very beneficial
* a fiducial bridge for passing fiducial locations seems to have been at the top of the list of features that should be added
