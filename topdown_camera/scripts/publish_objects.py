#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def publish_objects():
    pub = rospy.Publisher('test', String, queue_size=10)
    rospy.init_node('topdown_camera', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish('test')
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_objects()
    except rospy.ROSInterruptException:
        pass
