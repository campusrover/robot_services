#! /usr/bin/python
"""
this script pushes a list of commands to redis for the new movement_bridge to consume, for testing
"""
import rospy, redis, json
import bridge_tools as bt

commands = [
    # each entry in this list is in the pattern of cmd, distance or angle, duration and speed
    ["move", 2, 10, None],
    ["rotate", 90, 4, None],
    ["move", 15, None, None],
    ["move", 2, 10, None],
    ["stop", None, None, None],
    ["rotate", -180, 6, None],
    ["move", 15, 3, None],
    ["move", None, 4, 0.15],
    ["rotate", None, 3, 0.52],
    ["move", 5, 20, 0.13]
]
"""
expected behavior:
1. drive forward
2. 90 degree left turn (ccw)
3. error - not enough args
4. try to move forward
5. immediate interrupt
6. 180 dgree right turn (cw)
7. error - impossible speed
8. drive forward
9. rotate
10. error - incorrect math
"""
rospy.init_node("test_script")
redis = bt.redis_client()
key = bt.namespace_key("Cmd")
for c in commands: 
    cmd, delta, duration, speed = c
    package = {}
    if cmd:
        package["cmd"] = cmd
    if delta and cmd != "stop":
        if cmd == "move":
            package["distance"] = delta
        elif cmd == "rotate":
            package["angle"] = delta
    if duration:
        package["duration"] = duration
    if speed:
        package["speed"] = speed
    redis.rpush(key, str(json.dumps(package)))
    #rospy.sleep(3)