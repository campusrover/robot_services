#! /usr/bin/python
import rospy
import tf
import redis
import json
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
import tf2_ros as tf2 
from map_bridge import get_nearby_file

# pass a list of fiducials through redis. each fiducial is {"fid": x, "pose": {"location": [x, y, z], "orientation": [roll, pitch, yaw]}}. Pose is an absolute location in the odom TF frame. 

def fid_tf_cb(msg):
    global all_fids
    tfs = msg.transforms
    f_id_list = []
    if tfs:
        bcaster = tf2.TransformBroadcaster()
        for tf in tfs:
            # broadcast tf from camera to fid
            t2 = TransformStamped()
            t2.transform = tf.transform
            t2.header.stamp = rospy.Time.now()
            t2.header.frame_id = cam_frame  
            t2.child_frame_id = "fid{}".format(tf.fiducial_id)
            bcaster.sendTransform(t2)
            f_id_list.append("fid{}".format(tf.fiducial_id))

        for fid in f_id_list:
            try:
                # get the tf from the fiducial to the camera or odom (if available)
                fid_shift, fid_rot = listener.lookupTransform(frame, fid, rospy.Time(0))
                fid_rot = euler_from_quaternion(fid_rot)
                all_fids[fid] = {"fid": fid.strip("fid"), "pose": {"location": fid_shift, "orientation": fid_rot}}
            except:
                pass

    package = json.dumps({ 
        "fid_count": len(all_fids.keys()),
        "dict": fid_dict,  
        "frame": frame,
        "data": [all_fids[key] for key in all_fids.keys()]
    })
    redis.rpush(redis_key, str(package))
    with open(get_nearby_file("fiddump.json"), 'w') as save:
        json.dump(json.loads(package), save)

    #TODO remove this debug line:
    print("[FID BRIDGE] LIST OF SEEN FIDS: ", all_fids.keys())

def reset_cb(msg):
    # empty the list 
    while int(redis.llen(redis_key)) > 0:
        redis.lpop(redis_key)
    # push empty JSON
    package = json.dumps({ 
        "fid_count": 0,
        "dict": fid_dict, 
        "frame": "camera",
        "data": []
    })
    redis.rpush(redis_key, package)

if __name__ == "__main__":
    rospy.init_node("fiducial_bridge")
    r_serv = rospy.get_param("redis_server", "")
    r_port = rospy.get_param("redis_port", "")
    r_pass = rospy.get_param("redis_pw", "")
    if r_serv and r_port and r_pass:
        redis = redis.Redis(host=r_serv, port=int(r_port), password=r_pass)
    else:
        redis = redis.Redis()
    redis_key = "Fiducials"
    r_namespace = rospy.get_param("redis_ns", "")
    if r_namespace:
        redis_key = r_namespace + "/" + redis_key
    fid_tf_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fid_tf_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)

    # WARNING!!! this node needs to have know what the camera link is - via a param
    # for simulations, make sure to use model:=waffle_pi and cam_frame = camera_rgb_optical_frame
    cam_frame = rospy.get_param("fiducial_bridge/cam_frame", "camera")
    frame = cam_frame
    fid_dict = rospy.get_param("fiducial_bridge/dict", 7)
    all_fids = {}
    listener = tf.TransformListener()
    # main loop: get tf from odom to map
    while not rospy.is_shutdown():
        try:
            cam_shift, cam_rot = listener.lookupTransform("odom", cam_frame, rospy.Time(0))  # gives us the tf from odom to map. values inverted for tf from map to odom. 
            cam_rot = euler_from_quaternion(cam_rot)
            frame = "odom"
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue