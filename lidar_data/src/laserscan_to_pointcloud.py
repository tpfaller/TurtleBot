#!/usr/bin/env python

'''
This node subscribes to the LaserScan topic, converts it to PointCloud & publishes the data to a 'point_cloud' topic.
'''

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection


class LaserScanToPointCloud():

    def __init__(self):
        self.laser_proj = LaserProjection()
        self.pc2_pub = rospy.Publisher("/point_cloud", PointCloud2, queue_size=1)
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.laserscan_callback)

    def laserscan_callback(self, data):
        pc2_data = self.laser_proj.projectLaser(data)
        self.pc2_pub.publish(pc2_data)


if __name__ == '__main__':
    rospy.init_node("laserscan_to_pointcloud")
    node = LaserScanToPointCloud()
    rospy.spin()

