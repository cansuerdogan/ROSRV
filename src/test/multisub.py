#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from multitopics import *

def image_cb(msg):
  rospy.loginfo( "%s: I heard %s" % (rospy.get_caller_id(), type(msg) ))
def twist_cb(msg):
  rospy.loginfo( "%s: I heard %s" % (rospy.get_caller_id(), type(msg) ))
def string_cb(msg):
  rospy.loginfo( "%s: I heard %s" % (rospy.get_caller_id(), type(msg) ))

def listener():
  rospy.init_node('smooth_listener', anonymous=True)
  tname, ttype = get_topics_and_types()
  cblist = dict()
  cblist['geometry_msgs/TwistStamped'] = twist_cb
  cblist['sensor_msgs/Image'] = image_cb
  cblist['std_msgs/String'] = string_cb

  for t in tname.keys():
    tt = ttype[t]
    tts = tt._type
    cb = cblist[tts]
    rospy.loginfo( "Listening to [%s] of type %s" % (t, tts) )
    rospy.Subscriber(t, tt, cb)
    break
  rospy.spin()

if __name__ == '__main__':
  listener()
