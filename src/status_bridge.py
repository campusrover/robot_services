#!/usr/bin/python
import redis, rospy, json, psutil, datetime
from std_msgs.msg import Empty, String
from collections import OrderedDict
import bridge_tools as bt


def echo_cb(msg):
    global live_nodes
    print(msg.data)
    live_nodes.add(msg.data)

rospy.init_node("status_bridge")
redis = bt.redis_client()
redis_key = bt.namespace_key("Status")

bt.establish_reset(redis, redis_key, bt.reset, None, "status", "time", "cpu", "errors")
# every node will get a pulse sub and echo pub
bt.establish_pulse()
# these two are exclusive to the status bridge, because it sends the pulses and recieves the echoes
pulse_pub = rospy.Publisher("/pulse", Empty, queue_size=1)
echo_sub = rospy.Subscriber("/pulse_echo", String, echo_cb)

past_live_nodes = set()
live_nodes = set()
dead_nodes = set()
statuses = ["perfect", "excellent", "very good", "good", "not good", "bad", "very bad", "awful"]
wait_time = 15
end_time = rospy.get_time() + wait_time
pulse_pub.publish(Empty())  # send out a pulse
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    if rospy.get_time() > end_time:
        if past_live_nodes:
            persistant_nodes = live_nodes.intersection(past_live_nodes)  # nodes that were alive both in the most recent pulse and in the previous pulse
            if len(past_live_nodes) > len(persistant_nodes):  # more nodes in the past than persistant - a node has died
                dead_nodes = dead_nodes.union(past_live_nodes.symmetric_difference(persistant_nodes))
            if len(persistant_nodes) < len(live_nodes):  # more nodes in the present than persistant - a node may have been revived, or n\just a new node online
                new_nodes = live_nodes.symmetric_difference(persistant_nodes)
                dead_nodes = set([node for node in dead_nodes if node not in new_nodes])
            score = int(max(psutil.cpu_percent() - 80, 0) / 5) + len(dead_nodes)  # 1 point added to score for every 5 percent usage above 80 percent, 1 point added to score for each dead bridge node
            package = OrderedDict([
                ("time", datetime.datetime.now().isoformat()),
                ("status", statuses[min(score, 7)]),
                ("cpu", "{}%".format(psutil.cpu_percent())),  # TODO this is 0.0 often, which conflicts with "good" status messages indicating >95% cpu usage (because of gazebo)
                ("errors", list(dead_nodes))
            ])
            redis.set(redis_key, json.dumps(package))
        
        end_time = rospy.get_time() + wait_time
        past_live_nodes = live_nodes.copy()
        live_nodes.clear()
        pulse_pub.publish(Empty())  # send out a new pulse 
    rate.sleep()
