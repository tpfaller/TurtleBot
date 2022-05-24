#!/usr/bin/env python

'''
This node subscribes to the LaserScan topic and writes the data to a csv file.
'''

import rospy
from sensor_msgs.msg import LaserScan
import csv
import os


class LaserScanLogger:

    def __init__(self):
        self.csv_file = open(os.path.join(os.path.dirname ( __file__), os.path.pardir)  + '/data/laserscan_log.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)

        self.laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)

    def laserscan_callback(self, data):
        self.csv_writer.writerow(data.ranges)


if __name__ == '__main__':
    rospy.init_node('laserscan_logger', anonymous=True)
    node = LaserScanLogger()
    rospy.spin()

