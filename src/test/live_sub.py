#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    try:
      topic = rospy.get_param( "~chatter_topic" )
      rospy.loginfo( "Listening to [%s]" % topic )
      rospy.Subscriber(topic, String, callback)
      rospy.spin()
    except:
      print( "Failed to get parameters!" )
        
if __name__ == '__main__':
    listener()
