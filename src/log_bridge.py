#! /usr/bin/python
import redis, rospy, json
from rosgraph_msgs.msg import Log
from std_msgs.msg import Empty
from std_msgs.msg import Empty
from collections import OrderedDict
import bridge_tools as bt 

def log_cb (msg):
    global special_log_keys
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
        if whitelist.get(sender, 0) > 1 and msg.msg.startswith("[") and len(msg.msg.split(']')) == 2:
            parts = msg.msg.lstrip('[ ').split(']')
            info = parts[0]
            message = parts[1]
            key, code = info.split()
            package = json.dumps(OrderedDict([
                ("level", levels[msg.level]),
                ("from", sender),
                ("code", code.upper()),
                ("message", message)
            ]))
            key = bt.namespace_key('_'.join([x.capitalize() for x in key.split('_')])) # turns "key_name" to "Key_Name"
            redis.rpush(key, str(package))
            special_log_keys.add(key)
            bt.save_json_file("special_log_keys.json", list(special_log_keys))
        # otherwise, just send to the standard log key
        else:
            redis.rpush(redis_key, str(package))

def reset_cb(msg):
    # empty the list 
    while int(redis.llen(redis_key)) > 0:
        redis.lpop(redis_key)
    for key in special_log_keys:  # empty out all generated keys
        while int(redis.llen(key)) > 0:
            redis.lpop(key)
    # push empty JSON
    rospy.loginfo("Reset applied")



if __name__ == "__main__":
    global whitelist
    rospy.init_node("log_bridge")
    redis = bt.redis_client()
    redis_key = bt.namespace_key("Log")
    redis_key2 = bt.namespace_key("Log_Settings")

    roslog_sub = rospy.Subscriber('/rosout', Log, log_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)
    whitelist = bt.load_json_file("log_whitelist.json", dict())
    special_log_keys = set(bt.load_json_file("special_log_keys.json", []))
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        keys = redis.scan_iter(redis_key2 + "*")  # get all the keys under the ns/Log_Settings key domain
        update = False
        for k in keys:
            name = k.replace(redis_key2, '')  # strips <ns>/Log_Settings
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
            bt.save_json_file("log_whitelist.json", whitelist)
        rate.sleep()