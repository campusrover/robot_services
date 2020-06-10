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
            ("level", levels[msg.level]),
            ("from", msg.name),
            ("message", msg.msg)
        ]))
        redis.rpush(redis_key, str(package))

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
    redis_key2 = "Log_Edit"
    r_namespace = rospy.get_param("redis_ns", "")
    if r_namespace:
        redis_key = r_namespace + "/" + redis_key
        redis_key2 = r_namespace + "/" + redis_key2
    fid_tf_sub = rospy.Subscriber('/rosout', Log, log_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)
    try:
        try:  # first try to get namespaced whitelist
            with open(get_nearby_file('{}log_whitelist.json'.format(r_namespace)), 'r') as f:
                whitelist = set(json.load(f))
        except:  # then try to get generic whitelist
            with open(get_nearby_file('log_whitelist.json'), 'r') as f:
                whitelist = set(json.load(f))
    except:  # otherwise, whitelist is empty
        whitelist = set()
   
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if redis.llen(redis_key2) > 0:  # try to get a request
            req = redis.lpop(redis_key2)
            try:
                node_name, action = req.strip().split()  # unpack the request
                if not node_name.startswith("/"):
                    node_name = "/" + node_name
                if int(action):  # action == 1: add to list
                    whitelist.add(node_name)
                else:  # action == 0: remove from list
                    whitelist.remove(node_name)
                with open(get_nearby_file('{}log_whitelist.json'.format(r_namespace)), 'w') as f:  # save the whitelist to a namespaced file so Dave doesn't change Carl's whitelist
                    json.dump(sorted(list(whitelist)), f)
                    rospy.loginfo("{} {} Log Whitelist".format(node_name, "added to" if int(action) else "removed from"))
            except Exception as e:
                rospy.logerr("Could not complete request \"{}\" because of {} ".format(req, e))  # include request and error reason in error log
        rate.sleep()