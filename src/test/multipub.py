#!/usr/bin/env python

import rospy
from multitopics import *

def talker():
  rospy.init_node('smooth_talker')
  tname, ttype = get_topics_and_types()
  tpub = dict()
  pub_rate_hz = 1
  try:
    pub_rate_hz = rospy.get_param( "~pub_rate_hz" )
    rospy.loginfo( "param(%s)=%s" % ( "~pub_rate_hz", pub_rate_hz ) )
  except:
    pass
  for t in ttype.keys():
    topic_param = t + "_topic"
    topic_param = "~" + topic_param
    tpub[t] = rospy.Publisher(t, ttype[t])

  rate = rospy.Rate( pub_rate_hz )
  count = 0;
  while not rospy.is_shutdown():
    rospy.loginfo( 'publishing %d', count )
    count += 1
    for t in ttype.keys():
      msg = ttype[t]()
      tpub[t].publish( msg )
    rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass

