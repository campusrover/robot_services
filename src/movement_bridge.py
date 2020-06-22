#! /usr/bin/python
import rospy, json, actionlib, math, redis, time
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from tavares_padilha_line_merge import Point
import bridge_tools as bt 

def odom_cb(msg):
    global pose
    pose = msg.pose.pose

def scan_cb(msg):
    global flags
    # assumes a 360 degree lidar with 1 degree increments (hlds lidar present on TB3's)
    if flags["moving"]:
        if flags["forward"]:
            cone = msg.ranges[-45:] + msg.ranges[:45]
        else:
            cone = msg.ranges[135:225]
        if min(cone) < safe_thresh:
            flags["stalled"] = True
    else:
        flags["stalled"] = False
        

def execute_command(command_json):
    global flags, current_twist, expiration_time, current_cmd
    print(command_json)
    approved = True
    cmds = ["stop", "move", "rotate"]
    units = {"move":"distance", "rotate":"angle"}
    given_params = command_json.keys()
    speed = 0
    try:
        action = command_json["cmd"].lower().strip()
        if action != "stop":
            provided_args = set(["speed", "duration", "distance", "angle"]).intersection(set(given_params))
            if len(provided_args) < 2:
                rospy.logerr("[cmd_feedback invalid] not enough informaton given. for move or rotate please provide at least 2 of 3 [speed, duration, distance/angle]. Aborting.")
                approved = False
            elif len(set(["distance", "angle"]).intersection(provided_args)) > 1:
                rospy.logerr("[cmd_feedback invalid] Please provide distance or angle, not both. Aborting.")
                approved = False
            else:
                try:
                    duration = float(command_json.get("duration", 0))
                    speed = float(command_json.get("speed", 0))
                    delta = float(command_json.get(units[action], 0))
                except:
                    rospy.logerr("[cmd_feedback invalid] one or more non-float arguments given. Aborting")
                    approved = False
                    speed = 1  # set values to 1 just to guarantee no math errors happen 
                    duration = 1
                    delta = 1
                else:  # this block executes if the above except does not
                    if (not delta and not speed) or (not duration and not delta) or (not duration and not speed):  # check if more than 1 of the args are missing
                        rospy.logerr("[cmd_feedback invalid] more than one 0 value among [speed, duration, distance/angle]. Aborting")
                        approved = False
                    elif delta and speed and duration:  # check if all three args are given, that they are correct
                        calc_speed = delta/duration
                        if abs(calc_speed - speed) > 0.05:  # don't expect perfect matches, but expect them to be close
                            rospy.logerr("[cmd_feedback invalid] math error. speed does not match distance/duration. Aborting")
                            approved = False
                    else:  # if only one is missing, then calculate from the other two. don't calculate delta because it's not needed
                        if not duration:
                            duration = delta/speed
                        if not speed:
                            speed = delta/duration
                
        t = Twist()
        print("{} speed: {}".format(action, speed))  # TODO delete once debugged
        if action == 'rotate':
            speed *= math.pi/180  # do rotate
            if abs(speed) > 2.8:   # hard-coded tb3 speed 
                rospy.logerr("[cmd_feedback invalid] Bad command. angle and time not possible with max speed. Aborting.")
                approved = False
            else:
                t.angular.z = speed
        elif action== 'move':  # do move
            if abs(speed) > 0.22:   # hard-coded tb3 speed again
                rospy.logerr("[cmd_feedback invalid] Bad command. distance and time not possible with max speed. Aborting")
                approved = False
            else:
                t.linear.x = speed
                flags["moving"] = True
                if speed > 0:
                    flags["forward"] = True
                else:
                    flags["forward"] = False
        elif action == 'stop':
            approved = False  # somehow a stop might slip through the crack to current_cmd, in which case just don't do anything. 
        else:
            rospy.logerr("[cmd_feedback invalid] Invalid command given. current supported commands are: {}".format(cmds))
            approved = False
        if approved:
            rospy.loginfo("[cmd_feedback approved] command passed all checks. Beginning execution")
            current_twist = t
            expiration_time = rospy.get_time() + duration
        else:
            current_cmd = {}
    except Exception as e:
        rospy.logerr("[cmd_feedback invalid] Something unexpected occurred. Aborting. command: {} | exception: {}".format(json.dumps(command_json), e))
        current_cmd = {}

if __name__ == "__main__":
    rospy.init_node("movement_bridge")
    redis = bt.redis_client()
    redis_key = bt.namespace_key("Cmd")

    odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)
    scan_sub = rospy.Subscriber("/scan", LaserScan, scan_cb)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
    pose = None
    flags = {"moving": False, "forward": False, "stalled": False, "stopped": False}
    safe_thresh = 0.25
    current_twist = Twist()
    current_cmd = {}
    next_cmd = {}
    expiration_time = None
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if redis.llen(redis_key) > 0 and not next_cmd:
            next_cmd = json.loads(redis.lpop(redis_key))  # if next command slot is empty, then get a new commadn from redis
        if not current_cmd and next_cmd:  # if current command is empty and next command is not, then move the command to the current slot
            current_cmd = next_cmd
            next_cmd = {}
            if "cmd" not in current_cmd.keys():  # check to make sure the json at least has a cmd key
                rospy.logerr("[cmd_feedback invalid] cmd not given. No action available. Aborting.")
                current_cmd = {}
            else:
                execute_command(current_cmd)
        if current_cmd != None and next_cmd.get("cmd", "").strip().lower() == "stop":  # if there is a current command, but the next command is a stop, then stop the current action
            flags["stopped"] = True  # set the stopped flag
            expiration_time = rospy.get_time()  # 
            next_cmd = {}  # empty next command slot
        if expiration_time:
            if rospy.get_time() < expiration_time and not flags["stalled"]:
                cmd_pub.publish(current_twist)
                safe_thresh = current_twist.linear.x + 0.15
            else:
                expiration_time = None  # Remove expiration time
                cmd_pub.publish(Twist())  # publish empty twist to stop 
                current_cmd = {}  # remove current command slot
                current_twist = Twist()
                if flags["stalled"]:  # Failure
                    rospy.logerr("[cmd_feedback stalled] Encountered obstruction. Movement aborted")
                else:  # success?
                    if flags["stopped"]:
                        rospy.loginfo("[cmd_feedback stopped] command interrupted by stop command")
                    else:
                        rospy.loginfo("[cmd_feedback success] Destination reached")
                flags["moving"] = False  # no longer moving
                flags["stopped"] = False
                flags["stalled"] = False
        rate.sleep()
