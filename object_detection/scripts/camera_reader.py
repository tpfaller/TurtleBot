#!/usr/bin/env python
import functools
import time
import numpy as np
import cv2
from topics import CITopic, CameraTopic
import rospy
import rosbag
import pyrealsense2 as rs
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge


class ImageListener(object):

    def __init__(self, topic = None):
        self.topic = topic

    def handle_img(self, np_image, timestamp):
        pass

class RealsenseReader(object):

    def __init__(self, width = 640, height = 480, fps = 30, color_listener = None, depth_listener = None):
        self.color_listener = color_listener
        self.depth_listener = depth_listener
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def read(self):
        # Start streaming
        self.pipeline.start(self.config)
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                self.handle_frames(color_frame, depth_frame)
        finally:
            self.pipeline.stop()

    def handle_frames(self, color_frame, depth_frame):

        if color_frame is None or depth_frame is None:
            return
        
        color_profile = color_frame.get_profile()
        cvsprofile = rs.video_stream_profile(color_profile)
        color_intrin = cvsprofile.get_intrinsics()

        # Apply filter to fill the Holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)
        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)

        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        color_timestamp = color_frame.get_timestamp()
        depth_timestamp = depth_frame.get_timestamp()

        if self.color_listener is not None:
            self.color_listener.handle_img(color_image, color_timestamp)
        if self.depth_listener is not None:
            self.depth_listener.handle_img(depth_image, depth_timestamp)
        self.handle_images(color_image, depth_image, color_timestamp, depth_timestamp, color_intrin)

    def handle_images(self, color_image, depth_image, color_timestamp, depth_timestamp, color_intrin):
        pass

class RealsenseTopicReader(object):

    def __init__(self, width = 640, height = 480, fps = 30, color_listener = None, depth_listener = None):
        self.color_listener = color_listener
        self.depth_listener = depth_listener
        self.depth_msg = None
        self.color_info = None
        self.depth_info = None
        self.color_subscriber = None
        self.depth_subscriber = None
        self.color_info_subscriber = None
        # self.depth_info_subscriber = None
        self.bridge = CvBridge()

    def read(self):
        self.color_subscriber = rospy.Subscriber(CITopic.color_img.value, CompressedImage, self.handle_color_image)
        self.depth_subscriber = rospy.Subscriber(CameraTopic.depth_img.value, Image, self.handle_depth_image)
        # self.color_info_subscriber = rospy.Subscriber(CameraTopic.camera_info_depth.value, CameraInfo, self.handle_color_info)
        self.depth_info_subscriber = rospy.Subscriber(CameraTopic.camera_info_depth.value, CameraInfo, self.handle_depth_info)
        rospy.spin()

    def handle_color_image(self, color_msg):
        if self.depth_msg is not None and self.depth_info is not None:

            depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding='passthrough')

            color_np_arr = np.fromstring(color_msg.data, np.uint8)
            color_img = cv2.imdecode(color_np_arr, cv2.IMREAD_COLOR)

            color_timestamp = color_msg.header.stamp.to_sec()
            depth_timestamp = self.depth_msg.header.stamp.to_sec()
            intrinsics = rs.intrinsics()
            intrinsics.width = self.depth_info.width
            intrinsics.height = self.depth_info.height
            intrinsics.ppx = self.depth_info.K[2]
            intrinsics.ppy = self.depth_info.K[5]
            intrinsics.fx = self.depth_info.K[0]
            intrinsics.fy = self.depth_info.K[4]
            intrinsics.model  = rs.distortion.none     
            intrinsics.coeffs = [i for i in self.depth_info.D]
            if self.depth_info.distortion_model == 'plumb_bob':
                intrinsics.model = rs.distortion.brown_conrady
            elif self.depth_info.distortion_model == 'equidistant':
                intrinsics.model = rs.distortion.kannala_brandt4
            intrinsics.coeffs = [i for i in self.depth_info.D]

            self.handle_images(color_img, depth_img, color_timestamp, depth_timestamp, intrinsics)

    def handle_depth_image(self, depth_msg):
        self.depth_msg = depth_msg

    def handle_color_info(self, camera_info):
        self.color_info = camera_info

    def handle_depth_info(self, camera_info):
        self.depth_info = camera_info

    def handle_images(self, color_image, depth_image, color_timestamp, depth_timestamp, depth_intrin):
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
