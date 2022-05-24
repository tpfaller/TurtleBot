#!/usr/bin/env python

'''
This node subscribes to the PointCloud topic and writes the data to a csv file.
'''

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import csv
import os


class PointCloudLogger:

    def __init__(self):
        self.csv_file = open(os.path.join(os.path.dirname ( __file__), os.path.pardir)  + '/data/pointcloud_log.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)

        self.pc2_sub = rospy.Subscriber('/point_cloud', PointCloud2, self.pointcloud_callback, queue_size=1)

    def pointcloud_callback(self, data):
        assert isinstance(data, PointCloud2)
        points = point_cloud2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True)
        self.csv_writer.writerows(points)


if __name__ == '__main__':
    rospy.init_node('pointcloud_logger', anonymous=True)
    node = PointCloudLogger()
    rospy.spin()

