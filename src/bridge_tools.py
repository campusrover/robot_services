#! /usr/bin/python
"""
common code snippets used throughout robot_services
"""
import rospy, redis, sys, json
from std_msgs.msg import Empty, String
from os.path import dirname, realpath
from os import sep

def redis_client():
    """
    gets rosparams relevant to redis login, and returns the appropriate redis client object
    """
    r_serv = rospy.get_param("redis_server", "")
    r_port = rospy.get_param("redis_port", "")
    r_pass = rospy.get_param("redis_pw", "")
    if r_serv and r_port and r_pass:
        return redis.Redis(host=r_serv, port=int(r_port), password=r_pass)
    else:
        return redis.Redis()

def establish_pulse():
    """
    Does all the setup neccessary for a node to echo pulses sent by status_bridge
    Returns subscriber to /pulse
    """
    def pulse_cb(msg):
        echo_pub.publish(rospy.get_name())
    pulse_sub = rospy.Subscriber("/pulse", Empty, pulse_cb)
    echo_pub = rospy.Publisher("/pulse_echo", String, queue_size=3)
    return pulse_sub

def reset(redis, key, *fields):
    """
    reset for get-set keys
    """
    d = {}
    for f in fields:
        d[f] = None 
    redis.set(key, str(json.dumps(d)))
    pass

def reset_queue(redis, key, *fields):
    """
    reset for queue-based keys
    """
    while redis.llen(key) > 0:
        redis.lpop(key)
    d = {}
    for f in fields:
        d[f] = None 
    redis.rpush(key, str(json.dumps(d)))

def reset_del(redis, key, *fields):
    """
    reset by deletion
    """
    redis.delete(key)

def establish_reset(redis, key, reset_fn=None, extra_fn=None, *reset_fields):
    """
    Does all the  setup neccassary for a reset on a single key
    Returns subscriber to /reset
    """
    def reset_cb(msg):
        if reset_fn:
            reset_fn(redis, key, *reset_fields)
        else:
            reset_del(redis, key)
        if extra_fn:
            extra_fn()
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)
    return reset_sub


def namespace_key(key):
    """
    checks for a redis namespace rosparam, and returns correct namespaced key
    """
    r_namespace = rospy.get_param("redis_ns", "")
    if r_namespace:
        return r_namespace + "/" + key
    else:
        return key

def get_nearby_file(filename):
    """
    gets the absolute path to a file in the same directory as the script that calls this function

    Or, provide a relative file path to get the absolute file path to the given file
    """
    return dirname(realpath(sys.argv[0])) + sep + filename

def load_json_file(filename, default):
    """
    tries to load nearby json file. if files deosn't exist, returns default
    """
    try:
        with open(get_nearby_file(filename), 'r') as f:
            return json.load(f)
    except:
        return default

def save_json_file(filename, obj):
    """
    saves obj as a JSON in a location relative to the file that calls this function
    """
    with open(get_nearby_file(filename), 'w') as f:
            json.dump(obj, f)
