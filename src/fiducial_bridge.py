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
            # broadcast tf from camera to fid. This lets us look up the tf later and it does all the hairy math for us. 
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
                all_fids[fid] = {"fid": fid.strip("fid"), "pose": {"location": fid_shift, "orientation": fid_rot}}  # update list of all seen fiducials
            except:
                pass

    package = json.dumps({ 
        "fid_count": len(all_fids.keys()),
        "dict": fid_dict,  
        "frame": frame,
        "data": [all_fids[key] for key in all_fids.keys()]
    })
    # push to redis and save most recent json to disk
    redis.rpush(redis_key, str(package))
    with open(get_nearby_file("fiddump.json"), 'w') as save:
        json.dump(json.loads(package), save)

    rospy.logdebug("********* List of seen Fids", 1) #: ", all_fids.keys(), "CURRENT:", [t.fiducial_id for t in tfs])

def reset_cb(msg):
    # empty the list 
    while int(redis.llen(redis_key)) > 0:
        redis.lpop(redis_key)
    # push empty JSON
    package = json.dumps({ 
        "fid_count": 0,
        "dict": fid_dict, 
        "frame": frame,
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
    queue_size = rospy.get_param("redis_qs", 5)
    fid_tf_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fid_tf_cb)
    reset_sub = rospy.Subscriber("/reset", Empty, reset_cb)

    # WARNING!!! this node needs to have know what the camera link is - via a param
    # for simulations, make sure to use model:=waffle_pi and cam_frame = camera_rgb_optical_frame
    cam_frame = rospy.get_param("fiducial_bridge/cam_frame", "camera")
    frame = cam_frame
    fid_dict = rospy.get_param("fiducial_bridge/dict", 7)
    all_fids = {}  # store all seen fuducial poses in this dictionary
    listener = tf.TransformListener()
    rate = rospy.Rate(5)
    # main loop: get tf from odom to map
    while not rospy.is_shutdown():
        try:
            # Make sure that a tf from cam_frame to odom exists. this could be done 2 easy ways: 1. publish robot urdf using robot state publisher. 2. static transform broadcaster from base_footprint to cam_frame
            odom_tf = listener.lookupTransform("odom", cam_frame, rospy.Time(0))  
            frame = "odom"
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("tf from provided camera frame \"{}\" to odom does not exist".format(cam_frame))
            frame = cam_frame
            continue
        # trim queue size
        if redis.llen(redis_key) > queue_size:
            redis.lpop(redis_key)
        rate.sleep()