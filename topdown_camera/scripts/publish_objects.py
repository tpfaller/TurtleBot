#!/usr/bin/env python
import rospy
from topdown_camera.msg import ObjectPose, ObjectPoseArray


def publish_objects():
    pub = rospy.Publisher('/game_objects', ObjectPoseArray, queue_size=10)
    rospy.init_node('topdown_camera', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(ObjectPoseArray([
            ObjectPose('turtlebot', 0.1, 0.1, 0.1, 0.1, 0),
            ObjectPose('obstacle_0', 0.3, 0.3, 0.1, 0.2, 0),
            ObjectPose('obstacle_1', 0.1, 0.7, 0.3, 0.1, 0),
            ObjectPose('obstacle_2', 0.7, 0.7, 0.1, 0.1, 0),
            ObjectPose('Captain_America', 0.5, 0.5, 0.1, 0.1, 0),
            ObjectPose('Hulk', 0.1, 0.5, 0.1, 0.1, 0),
            ObjectPose('Iron_Man', 0.8, 0.2, 0.1, 0.1, 0)]))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_objects()
    except rospy.ROSInterruptException:
        pass
