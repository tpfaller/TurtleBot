#!/usr/bin/env python
import rospy
from topdown_camera.msg import ObjectPose


def publish_objects():
    pub = rospy.Publisher('test', ObjectPose, queue_size=10)
    rospy.init_node('topdown_camera', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(ObjectPose('test', 0.3, 0.3, 0.1, 0.2, 0))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_objects()
    except rospy.ROSInterruptException:
        pass
