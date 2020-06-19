#! /usr/bin/python
"""
common code snippets used throughout robot_services
"""
import rospy, redis, sys, json
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
    with open(get_nearby_file(filename), 'w') as f:
            json.dump(obj, f)
