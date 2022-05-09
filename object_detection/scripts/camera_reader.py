#!/usr/bin/env python
import functools
import time
import numpy as np
import cv2
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage


class ImageListener(object):

    def __init__(self, topic):
        self.topic = topic

    def handle_img(self, np_image, timestamp):
        pass


class CIReader(object):

    def __init__(self, listeners = []):
        self.listeners = {}
        for l in listeners:
            topic_list = self.listeners.get(l.topic)
            if topic_list is None:
                self.listeners[l.topic] = [l]
            else:
                topic_list.append(l)            

    def read(self):
        pass

    def handle_ci_msg(self, topic, msg, timestamp = None):
        try:
            timestamp = msg.header.stamp.to_sec()
        except:
            if timestamp is None:
                timestamp = time.time()

        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        listeners = self.listeners.get(topic, [])
        for l in listeners:
            l.handle_img(image_np, timestamp)


class CISubscriber(CIReader):

    def __init__(self, listeners=[], rosnode_name = 'camera_reader'):
        super(CISubscriber, self).__init__(listeners)
        self.rosnode_name = rosnode_name

    def read(self):
        super(CISubscriber, self).read()
        rospy.init_node(self.rosnode_name, anonymous=True)
        subscribers = {}
        for topic in self.listeners.keys():
            subscribers[topic] = rospy.Subscriber(
                name=topic,
                data_class=CompressedImage,
                callback=functools.partial(self.handle_ci_msg, topic))
            print('Subscribed to %s' % topic)
        rospy.spin()


class CIBagReader(CIReader):
    
    def __init__(self, filename, listeners=[]):
        super(CIBagReader, self).__init__(listeners)
        self.filename = filename

    def read(self):
        super(CIBagReader, self).read()

        print('Reading file: %s' % self.filename)

        bag = rosbag.Bag(self.filename)
        bagContents = bag.read_messages(topics=self.listeners.keys())

        for topic, msg, t in bagContents:
            self.handle_ci_msg(topic, msg, t.to_sec())

        print('Finished reading file: %s' % self.filename)
