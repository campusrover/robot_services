#! /usr/bin/python
import redis, rospy
from rosgraph_msgs.msg import Log

def log_cb (msg):
    levels = {0: "DEBUG", 2:"INFO", 4:"WARN", 8:"ERROR", 16:"FATAL"}
    logpack = "[{}] from {}: ".format(levels[msg.level], msg.name)
    logpack += msg.msg
    redis.rpush(redis_key, logpack)

def reset_cb(msg):
    # empty the list 
    while int(redis.llen(redis_key)) > 0:
        redis.lpop(redis_key)
    # push empty JSON
    logpack = "[INFO] from log_bridge: reset applied"
    redis.rpush(redis_key, logpack)

if __name__ == "__main__":
    rospy.init_node("log_bridge")
    r_serv = rospy.get_param("redis_server", "")
    r_port = rospy.get_param("redis_port", "")
    r_pass = rospy.get_param("redis_pw", "")
    if r_serv and r_port and r_pass:
        redis = redis.Redis(host=r_serv, port=int(r_port), password=r_pass)
    else: 
        redis = redis.Redis()
    redis_key = "Log"
    fid_tf_sub = rospy.Subscriber('/rosout', Log, log_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)
   
    rospy.spin()