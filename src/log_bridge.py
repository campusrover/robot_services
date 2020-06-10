#! /usr/bin/python
import redis, rospy, json
from rosgraph_msgs.msg import Log
from std_msgs.msg import Empty
from map_bridge import get_nearby_file
from std_msgs.msg import Empty
from collections import OrderedDict

def log_cb (msg):
    levels = {1: "DEBUG", 2:"INFO", 4:"WARN", 8:"ERROR", 16:"FATAL"}
  # if the whitelist is empty, publish everyhting. If the whitelist is populated, only publish from approved nodes
    if (not whitelist) or msg.name in whitelist:
        package = json.dumps(OrderedDict([
            "level": msg.level,
            "from": msg.name,
            "message": msg.msg
        })

def reset_cb(msg):
    # empty the list 
    while int(redis.llen(redis_key)) > 0:
        redis.lpop(redis_key)
    # push empty JSON
    rospy.loginfo("Reset applied")

if __name__ == "__main__":
    global whitelist
    rospy.init_node("log_bridge")
    r_serv = rospy.get_param("redis_server", "")
    r_port = rospy.get_param("redis_port", "")
    r_pass = rospy.get_param("redis_pw", "")
    if r_serv and r_port and r_pass:
        redis = redis.Redis(host=r_serv, port=int(r_port), password=r_pass)
    else: 
        redis = redis.Redis()
    redis_key = "Log"
    r_namespace = rospy.get_param("redis_ns", "")
    if r_namespace:
        redis_key = r_namespace + "/" + redis_key
    fid_tf_sub = rospy.Subscriber('/rosout', Log, log_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)
    try:
        with open(get_nearby_file('log_whitelist.json'), 'r') as f:
            whitelist = set(json.load(f))
    except:
        whitelist = set()
   
    rospy.spin()