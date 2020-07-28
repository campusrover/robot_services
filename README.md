# ROS robot service bridges

a custom package for sending certain ROS information to any other app through REDIS. this package provides channels that are aimed to encourage merging robots with non robotic applictions. These channels will allow anyone wwith little or no programming experience, and no ROS experience, to be able to interact with robots for specific needs.

## Installation and setup

This package runs on Python 2.7 in ROS Melodic. 

Besides default packages that are included with python and ros installations, the redis python module is required. Get it with `pip install redis`. To communicate with redis on your local device instead of a redis server, use `apt install redis-server`.

The turtlebot3_slam package is currently required. clone it from [here](https://github.com/ROBOTIS-GIT/turtlebot3)

To use fiducials, add them with `apt install ros-melodic-fiducials` - this will install aruco_detect and a few other fiducial related packages.

To easily spawn multiple fiducials to gazebo, try the [Gazebo Fiducial Spawner Package](https://github.com/NateDimick/gazebo_fiducial_spawner)

`fidtest.launch` is provided to test fiducial recognition and localization in isolation from other parts of the bridge.

`maptest.launch` and `map_bridge_debug.py` are provided to test map generation in isolation. To run you will need PIL. Get it with `pip install Pillow`. `map_bridge_debug.py` draws a map based on the line segments it generates to help visualize while debgging.

## REDIS channels

1. "Map": Map JSON is `RPUSH`ed to this topic by `map_bridge.py`. `LPOP` should be used to retrieve Map JSON from Redis. Map data is formatted as follows:
    * `width` and `height` (in meters) 
    * `line_count` representing the number of line segments in the JSON
    * `origin`: a two-tuple of floats describing where the origin (0, 0) is relative to width and height - so if (w,h) = (2, 5) and origin = (1, 3) then the origin 1 unit from the left end of the width and 3 units from the bottom of the height
    * `data`, a list of line segments, represented as four-tuples representing the two endpoints of each line, [x1, y1, x2, y2]. line segment coordinates are in meters relative to the center of odometry if a TF from odom to map is available, otherwise they are relative to map center defined by origin from the map topic.
2. "Odom": Odom JSON is `SET` by `odom_bridge.py`. JSON includes: 
    * `odom_id` and `real_odom_id`: the sequential id of the odom updates sent to redis and updates generated in ROS, respectively
    * `location` as [x, y, z]
    * `orientation` as [roll, pitch, yaw]
    * `linearvelocity` in m/s
    * `angularvelocity` in rad/s. robot location is in meters relative to the center of odometry. Orientation is in radians.
3. "Bridge_Reset": is expected to be `SET` by external applications, should be either `0` or `1`. `1` indicates a request for a reset of all Redis keys. Keys will be reset with their normal JSON structure, with 0 values in each field. After key reset occurs, the value of this key will be `SET` to `0` by `reset_bridge.py`
4. "Cmd": `RPUSH` JSON to this key from a non-ROS application to inititate movement of the robot. Commands are `LPOP`ed and interpreted by `movement_bridge.py`. Command JSON should contain the following:
    * `cmd`, A string that says `stop` `move` or `rotate`
    * `distance` if `move` and `angle` if `rotate`: the amount of movement desired in meters or degrees
    * `duration`, the amount of time to move for in seconds
    * `speed` the speed to travel at in m/s or deg/s
    * Besides `cmd`, two of the wo of the other three must be provided
5. "Log": JSON `RPUSH`ed to this key. Only forwards logs from whitelisted nodes, by node name. see `src/log_whitelist.json`. JSON includes the following fields:
    * `level`: a string describing the type of log
    * `from`: the name of the nod ethat logged this message
    * `message`: the body of the message
6. "Log_Settings": see the section below on Updating Log Settings
7. "Fiducials": JSON that is `SET` by `fiducial_bridge.py` JSON includes info like:
    * `fid_count` representing the number of known fiducials
    * `dict` representing the fiducial marker format (see aruco marker documentation) 
    * `frame`, which will be `odom` if the transform from the camera link to odom is available, otherwise it will be `camera`, indicating that all values in the fiducial list are in coordinates relative to the robot, not the center of odometry. 
    * `data` is a list of known fiducial markers, where each element has a `fid` representing the marker's id number, and a `pose` which has `location` (in meters) and `orientation` components (in euler radians)
8. "Lidar": JSON is `SET` by `lidar_bridge.py` containing simplified lidar data - the shortest range in each slice of it's fov.
    * `max_range` and `min_range` in meters
    * `fov`: "field of view", in radians
    * `slices`: the number of pices the lidar has been broken into
    * `data`: the closest range in each slice, from front going ccw
9. "Status": JSON is `SET` by `status_bridge.py` with the following fields:
    * `time`: the ros time at the time of the update
    * `status`: a string that generally describes the status of robot_services based on a set of input factors
    * `cpu`: the measured cpu usage % 
    * `errors`: a list of nodes that have died while robot_services has been running. nodes that die upon launch won't be recorded.
10. "Kirby": `LPOP`ed by `kirby_listener.py`. See section Kirby Movement Commands for more information

## Launch

`bridge.launch` is the primary launch file for this package. Presently, it launches the map, odometry, reset, and movement command bridge nodes alongside SLAM. for the map bridge to work best, it needs a transform from `odom` to `map` to exist, which is provided by SLAM and AMCL (SLAM was chosen in this instance). `bridge.launch` also has the following launch arguments:

* `pose_update_thresh` (float): This arg is used to change the rate at which keys with information related to geometrical poses (like fiducials and odometry) are `set` to redis. Change is often measured in total absolute change from the past pose in terms of position and orientation (euler). A new update will only be sent once the total change since the past send has exceeded the threshold. Suggested values for this are between 0.1 and 0.5. The default value is 0.3. A value of 0 means odom updates are sent each time they are received.
* `ns` (string): *n*ame*s*pace. Namespaces all redis keys to \<ns>/\<key>
* `queue_size` (int): *q*ueue *s*ize. The maximum length for queue-based keys like `Map`. defaults to 7
* `comm` (string). Defaults to `server`. If set to any value besides server, then the redis server will be your local machine. Useful for debugging and not clogging up a remote redis server. 
* `lidar_slices` (int): the number points desired from lidar data. Max value 360.
* `urdf_file` the path to the urdf file of the robot that you are using. This is used to set the `robot_description` rosparam. if you are running robot_services alongside gazebo, then this arg does not need to be filled. If your robot's bringup already includes a urdf and robot_state_publisher, then this arg does not need to be filled. 

Use `roslaunch robot_services maptest.launch` to test the map bridge on any map. A few samples are included in this repo and the performance is logged in test.launch. the python module Pillow is required for test.launch.

### redis_login.yaml

to launch using a remote redis server, the file `redis_login.yaml` must be present in `/launch` and contain values for the rosparams `redis_server`, `redis_port` and `redis_pw`

## Bridge Tools

`src/bridge_tools.py` is provided to make developing new "bridges" quick and easy within the established framworks of Robot Services. Some example of how the tools provided make integrating new bridge nodes fast and easy.

`bridge_tools` is internally referred to and imported as `bt`

### Namespacing

To prevent collisions of multiple users on the same redis server, this package provides namespacing. Set the rosparam `redis_ns` to your namespace, and as a result, any of the channels above will be prefixed with your namespace. For example, a user with the namespace "greg" will have all of their redis channels prefixed with "greg/", e.g. "greg/Map", "greg/Odom" . . .

Refer to the `ns` launch arg below to easily set a personal namespace

use `redis_key = bt.namespace_key("key_name")` to namespace your redis keys


### Reset

Every new bridge node should support the reset operation. A reset can be requested by setting the key `Bridge_Reset` to `1`. On the ROS side, this proliferates a reset command via the `/reset` topic using a `std_msgs/Empty` message type. All each node has to do is subscribe to `/reset` and if they receive a message, their callback should reset all of the node's respective keys. Resets will delete all keys published by any active bridge nodes.

Set up a basic reset handler in a bridge node with `bt.establish_reset(redis_client_instance, *redis_keys)`. `bt.establish_reset` is pretty flexible for single-key resets. See examples of how it is used throughout the package. 

For some nodes, `bt.establish_reset` might not be robust enough, especially if new keys are dynamically created and removed during runtime.

### Status and "Pulse"

Like Reset, every bridge node needs to support "Pulse" and "Echo". Status_Bridge keeps track of which nodes are "alive" and does this by sending an empty message to `/pulse` every thirty seconds. Other nodes should send an echo in response to a pulse in the form of a `std_msgs/String` containing the name of the node as a response to each pulse to the topic `/pulse_echo`. 

Any node can setup pulse response with `bt.establish_pulse()`

## Kirby Movement Commands

General commands supported:

* go forward (X) - moves the robot to a location x (or default of 1.0) meters ahead of where it currently is.
* go to X Y - moves the robot to the given (x,y) coordinate on its map.
* turn left (X) - turns the robot x (or default of 90) degrees to the left.
* turn right (X) - turns the robot x (or default of 90) degrees to the right.
* patrol - tells the robot to explore its environment.

For more detailed documentation on commands, controls, and feedback please read this documentation on [movement and feedback](robot_movement_and_feedback.txt)

## Updating Log Settings

the "Log" and "Feedback" keys operate on a whitelist system. If `src/log_whitelist.json` does not exist, then every node will have basic "Log" priviledge. Otherwise, there is a tiered system as follows:

* nodes with whitelist level `0` are not whitelisted. 
* nodes with whitelist level `1` can send their logs to the "Log" key
* nodes with whitelist level `2` have level `1` priviledges and can create new keys to send custom logs to (see the next section - custom logging keys)

`src/log_whitelist.json` stores key/value pairs where the keys are the names of nodes and the values are their whitelist level. Please note that node names start with a `/`. 

To edit the whitelist remotely from redis, create a key called `<ns>/Log_Settings/<node_name>`, where:

* `ns` is your personal namespace
* `node_name` is the name of the node you want to change logging access for, e.g. `map_bridge` or `fiducial_bridge` (NB that the `/` in the node name is included in the key structure)

to this key, `SET` either 0, 1 or 2 to update that key's whitelist level

### Example

to change `lidar_bridge.py` to gain custom access, use `set mynamespace/Log_Settings/lidar_bridge 2`

## Custom Logging Keys

For a level-2 whitelist node to create their own keys, they must begin a log message with this pattern: `[<key> <code>]` where `key` is the name of the desired redis key to send this message to, and `code` is some sort of uniform string that describes the type of message. both `key` and `code` must **not** contain spaces. 

`key` will be capitalized (`Key`) and `code` will be uppercased (`CODE`).

JSON gets `RPUSH`ed to custom log keys. Custom log keys are subject to resets. Custom log keys are stored in `src/special_log_keys.json`

### Example

if the following line is inside `example.py` and `example` has logging level 2 powers:

``` python
rospy.loginfo("[example_demo all_ok] creating a new key is easy")
```

Then `lpop <ns>/Example_Demo` will return: 

``` sh
"{\"level\": \"INFO\", \"from\": \"/example\", \"code\": \"ALL_OK\", \"message\": \" creating a new key is easy\"}"
```

## Loose Ends and Areas for Improvement

For an overview of the strategy and algorithms used in the map bridge, please read [this markdown file](map_bridge.md)

TODO list of future features:

* generic_bridge.py was initially created with hopes of creating a generic subsccriber-publisher from ros to redis and vice-versa that would perhaps make use of rospy.msg.AnyMsg.
* consolodate_lines in map_bridge tends to chug and take a lot of time when faced with many lines. additionally, the process of making a numpy array of an occupancy grid can be very slow if the occupancy grid is large (see basement_map_gen4 in cr_ros_3). Speeding up these operations would be very beneficial
