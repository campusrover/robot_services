#! /usr/bin/python
import rospy
import tf
import redis
import json
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
import tf2_ros as tf2 
import bridge_tools as bt
from collections import OrderedDict


# pass a list of fiducials through redis. each fiducial is {"fid": x, "pose": {"location": [x, y, z], "orientation": [roll, pitch, yaw]}}. Pose is an absolute location in the odom TF frame. 
def fid_tf_cb(msg):
    global all_fids
    tfs = msg.transforms
    f_id_list = []
    update = False
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
                if fid not in all_fids.keys():  # automatically add new fids
                    all_fids[fid] = {"fid": fid.strip("fid"), "pose": {"location": fid_shift, "orientation": fid_rot}}  # update list of all seen fiducials
                    update = True
                else:  # only update a pose if it is significantly different
                    old_shift = all_fids[fid]["pose"]["location"]
                    old_rot = all_fids[fid]["pose"]["orientation"]
                    shift_diff = sum([abs(i-j) for i, j in zip(old_shift, fid_shift)])
                    rot_diff = sum([abs(i-j) for i, j in zip(old_rot, fid_rot)])
                    if shift_diff + rot_diff > pose_update_thresh:
                        all_fids[fid] = {"fid": fid.strip("fid"), "pose": {"location": fid_shift, "orientation": fid_rot}}  # update list of all seen fiducials
                        update = True
            except:
                pass
    if update:
        package = json.dumps(OrderedDict([ 
            ("fid_count", len(all_fids.keys())),
            ("dict", fid_dict),  
            ("frame", frame),
            ("data", [all_fids[key] for key in all_fids.keys()])
        ]))
        # push to redis and save most recent json to disk
        redis.set(redis_key, str(package))
        bt.save_json_file("fiddump.json", json.loads(package))

def fiducial_reset():
    global all_fids
    all_fids = {}

if __name__ == "__main__":
    rospy.init_node("fiducial_bridge")
    redis = bt.redis_client()
    redis_key = bt.namespace_key("Fiducials")

    queue_size = rospy.get_param("redis_qs", 5)
    fid_tf_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fid_tf_cb)
    bt.establish_reset(redis, redis_key, bt.reset, fiducial_reset, "data", "frame", "dict", "fid_count")
    bt.establish_pulse()

    # WARNING!!! this node needs to have know what the camera link is - via a param
    # for simulations, make sure to use model:=waffle_pi and cam_frame = camera_rgb_optical_frame
    cam_frame = rospy.get_param("fiducial_bridge/cam_frame", "camera")
    frame = cam_frame
    fid_dict = rospy.get_param("fiducial_bridge/dict", 7)
    pose_update_thresh = rospy.get_param("pose_update_thresh", 0)
    all_fids = {}  # store all seen fuducial poses in this dictionary
    listener = tf.TransformListener()
    tf_present = True
    rate = rospy.Rate(5)
    # main loop: get tf from odom to map
    while not rospy.is_shutdown():
        try:
            # Make sure that a tf from cam_frame to odom exists. this could be done 2 easy ways: 1. publish robot urdf using robot state publisher. 2. static transform broadcaster from base_footprint to cam_frame
            odom_tf = listener.lookupTransform("odom", cam_frame, rospy.Time(0))
            if not tf_present:
                rospy.loginfo("fiducial camera tf to odom resolved")
            tf_present = True
            frame = "odom"
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if tf_present:
                rospy.logwarn("tf from provided camera frame \"{}\" to odom does not exist".format(cam_frame))
                odom_exists = listener.frameExists("odom")
                cam_exists = listener.frameExists(cam_frame)
                rospy.logwarn("odom does {} exist. {} does {} exist.".format("" if odom_exists else "not", cam_frame, "" if cam_exists else "not"))
                tf_present = False
            frame = cam_frame
            continue
        rate.sleep()
        