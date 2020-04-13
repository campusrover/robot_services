#! /usr/bin/python
import rospy
import redis

# the idea here is it would be an object that would do all the converting to redis json automatically

class message_bridge:
    def __init__(self, topic, type=AnyMsg, callback=None):
        pass
    def 