#! /usr/bin/python
import redis, rospy, json
from rosgraph_msgs.msg import Log
from std_msgs.msg import Empty
from map_bridge import get_nearby_file
from std_msgs.msg import Empty
from collections import OrderedDict

def log_cb (msg):
    levels = {1: "DEBUG", 2:"INFO", 4:"WARN", 8:"ERROR", 16:"FATAL"}
    # for reference: whitelist levels are 0: banned: 1: able to send to /Log, 2: able to send to /Log and /Feedback
    sender = msg.name
    # if the whitelist is empty, publish everything. If the whitelist is populated, only publish from approved nodes
    if (not whitelist) or whitelist.get(sender, 0) > 0:
        package = json.dumps(OrderedDict([
            ("level", levels[msg.level]),
            ("from", sender),
            ("message", msg.msg)
        ]))
        # if this is a tagged feedback message and the sending node has feedback priviledge, then send to the feedback key
        if whitelist.get(sender, 0) > 1 and '[feedback]' in msg.msg.lower().strip().split():
            redis.rpush(redis_key3, str(package).replace('[feedback] ', ''))
        # otherwise, just send to the standard log key
        else:
            redis.rpush(redis_key, str(package))

def reset_cb(msg):
    # empty the list 
    while int(redis.llen(redis_key)) > 0:
        redis.lpop(redis_key)
    while int(redis.llen(redis_key3)) > 0:
        redis.lpop(redis_key3)
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
    redis_key2 = "Log_Settings"
    redis_key3 = "Feedback"
    r_namespace = rospy.get_param("redis_ns", "")
    if r_namespace:
        redis_key = r_namespace + "/" + redis_key
        redis_key2 = r_namespace + "/" + redis_key2
        redis_key3 = r_namespace + "/" + redis_key3
    fid_tf_sub = rospy.Subscriber('/rosout', Log, log_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)
    try:
        with open(get_nearby_file('log_whitelist.json'), 'r') as f:
            whitelist = json.load(f)
    except:  # otherwise, whitelist is empty
        whitelist = dict()
   
    rate = rospy.Rate(1)
    print('Hello world')
    while not rospy.is_shutdown():
        keys = redis.scan_iter(redis_key2 + "*")  # get all the keys under the ns/Log_Settings key domain
        update = False
        for k in keys:
            name = k.replace(redis_key2, '')
            value = redis.get(k)
            curr_value = whitelist.get(name, -1)
            if value != curr_value:  # check the redis value against the stored whitelist value. if they differ, then update the whitelist. 
                try:
                    assert(value in ["0", "1", "2"])
                    whitelist[name] = int(value)
                    update = True
                except:
                    rospy.logerr("invalid whitelist code [{}] given for node {}".format(value, name))
                    redis.set(k, curr_value)  # if an invalid value was passed, then reset the redis key value to what was in the whitelist
        if update:
            with open(get_nearby_file('log_whitelist.json'), 'w') as f:
                json.dump(whitelist, f)
        rate.sleep()