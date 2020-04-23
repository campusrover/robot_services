# ROS-Voxelworld (unity) bridge

a custom package for sending certain ROS information to any other app through REDIS

## REDIS channels

1. "Map": Map JSON is `RPUSH`ed to this topic by `map_bridge.py`. Map data is sent as the dimentions of the map in `width` and `height` (in meters), a `line_count` representing the number of line segments in the JSON, and `data`, a list of line segments, represented as four-tuples representing the two endpoints of each line, [x1, y1, x2, y2]
2. "Odom": Odom JSOn is `SET` by `movement_bridge.py`. JSON includes `location` as [x, y, z], `orientation` as [roll, pitch, yaw], `linearvelocity` in m/s, `angularvelocity` in rad/s
3. "cmd": read by `movement_bridge.py` to get movement commands from the other simulation
