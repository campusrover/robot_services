#! /usr/bin/python
import rospy, redis
from std_msgs.msg import Empty
import bridge_tools as bt

if __name__ == "__main__":
    rospy.init_node("reset_bridge")
    redis = bt.redis_client()
    redis_key = bt.namespace_key("Bridge_Reset")


    reset_pub = rospy.Publisher("/reset", Empty, queue_size=10)
    bt.establish_pulse()

    hz = 2
    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        # try to check the redis reset key
        try:
            reset = int(redis.get(redis_key))
        except:
            reset = 0
        # if reset key is not 0, then proliferate reset command
        if reset:
            reset_pub.publish(Empty())
            # wait for about 1 second, that's all it should take to reset all keys, right?
            for i in xrange(max(1, hz)):
                rate.sleep()
            # indicate to outside applications that redis has been reset
            redis.set(redis_key, str(0))
        rate.sleep()
