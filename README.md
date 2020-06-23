# cmd test branch readme

in this branch, for the usual readme please refer to [masterREADME.md](masterREADME.md)

## how to run

1. launch a gazebo world
2. launch `robot_services move-test.launch`, which includes `log_bridge`, `move_test_script` and the new `movement_bridge`

## how to modify 

`src/move_test_script.py` is a python script that will automatically push a list of cmd to redis in one batch. to delay or stagger command sending, uncomment the final line of the script and set the duration of `rospy.sleep()` to your desired amount of time between `RPUSH`es.

at the top of the file is a list called `commands`, with included documentation to help guide creating new commands. 

## failure conditions

Here are the expected failure conditions and examples of how to create them in the `commands` list of `move_test_script`

1. no `cmd` key `[None, 4, 6, 20]`
2. unsupported `cmd` key `["dance", 8, 24, .2]`
3. non-float for duration, speed or angle/distance `["move", "vanilla", 6, .15]`
4. More than one absent non-`cmd` argument `["rotate", 90, None, None]`
5. speed that exceeds maximum possible robot speed (currently hardcoded to tb3 burger spec) `["move", 10, 4, None]`
6. all three args are given but the math is wrong, speed != distance/time (or isn't within an error tolerance) `["move", 10, 6, 1.3]`]
7. both `distance` and `angle` fields are included in the json - not really possible to create this condition in the test script

there is also an unexpected error message, in this case both the command and the exception will be included in the error message.

## feedback

`log_bridge` is included so that `movement_bridge` errors and other logs can be viewed under the `<ns>/Cmd_Feedback` key

if a command does not throw an `INVALID` code on a command, then it will throw an `APPROVED` code and begin moving. `APPROVED` should always be followed by one of the three following:

* `STOPPED` if interrupted by a `stop` `cmd`
* `STALLED` if lidar detects an obstacle in its path
* `SUCCESS` if not `STALLED` or `STOPPED`