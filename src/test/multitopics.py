#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image

def get_topics_and_types():
  ttype = dict()
  ttype['mumble'] = String
  ttype['turn'] = TwistStamped
  ttype['camera'] = Image
  tname = dict()
  for t in ttype.keys():
    topic_param = t + "_topic"
    tname[t] = t
    try:
      t = rospy.get_param( topic_param ) 
      rospy.loginfo( "param(%s)=%s" % ( topic_param, t ) )
    except:
      pass
    try:
      t = rospy.get_param( topic_param )
      rospy.loginfo( "param(%s)=%s" % ( topic_param, t ) )
    except:
      pass
  return tname, ttype


