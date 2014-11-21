#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
  rospy.init_node('talker')
  topic = rospy.get_param( "/chatter_topic" ) 
  pub_rate_hz = rospy.get_param( "~pub_rate_hz" )
  try:
    topic = rospy.get_param( "~chatter_topic" )
    rospy.loginfo( "param chatter_topic=%s" % chatter_topic )
  except:
    pass
  pub = rospy.Publisher(topic, String)
  rate = rospy.Rate( pub_rate_hz )
  while not rospy.is_shutdown():
    str = "hello world %s" % rospy.get_time()
    rospy.loginfo(str)
    pub.publish(String(str))
    rate.sleep()


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
