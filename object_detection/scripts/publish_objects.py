#!/usr/bin/env python
import math
import sys
from camera_reader import CIBagReader, ImageListener, RealsenseReader, RealsenseTopicReader
from topics import CITopic
import rospy
from object_detection.msg import ObjectPosition, ObjectPositionArray
from multiprocessing.connection import Client
import cv2
import pyrealsense2 as rs

# region: testing

COLORS = {
    'iron_man': (255, 0, 0),
    'hulk': (0, 255, 0),
    'captain_america': (0, 0, 255),
    'obstacles': (0, 0, 0),
    'wall': (100, 100, 100)
}

class TestListener(ImageListener):

    def __init__(self, object_publisher, skip_frames = 0):
        super(TestListener, self).__init__(CITopic.color_img.value)
        self.client = Client(('localhost', 6000))
        self.object_publisher = object_publisher
        self.client.send('turtlebot')
        self.skip_frames = skip_frames
        self.skipped = 0

    def handle_img(self, np_image, timestamp):
        if self.skipped < self.skip_frames:
            self.skipped += 1
            return
        super(TestListener, self).handle_img(np_image, timestamp)
        self.client.send(np_image)
        object_list = self.client.recv()

        img = np_image

        height = len(np_image)
        width = 0
        if height > 0:
            width = len(np_image[0])

        detected_objects = []
        for obj_id, pos in object_list:
            distance = 10 # test value
            angle = 0 # test value
            color = COLORS.get(obj_id, (0, 255, 255))
            img = cv2.circle(img, (int(pos[0] * width), int(pos[1] * height)), 5, color, thickness=-1)
            detected_objects.append(ObjectPosition(obj_id, distance, angle))

        if self.object_publisher is not None:
            self.object_publisher.publish(ObjectPositionArray(detected_objects))

        cv2.imshow('objects', img)
        # cv2.waitKey(0)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            sys.exit(0)

# endregion

class ObjectDetection(RealsenseTopicReader):

    def __init__(self, server_address = ('localhost', 6000), object_publisher = None, width=640, height=480, fps=30):
        super(ObjectDetection, self).__init__(width, height, fps, None, None)
        self.client = Client(server_address)
        self.object_publisher = object_publisher
        self.client.send('turtlebot')

    def handle_images(self, color_image, depth_image, color_timestamp, depth_timestamp, color_intrin):
        super(ObjectDetection, self).handle_images(color_image, depth_image, color_timestamp, depth_timestamp, color_intrin)
        self.client.send(color_image)
        object_list = self.client.recv()
        print('Detected %d objects' % len(object_list))
        detected_objects = []
        d_height = len(depth_image)
        d_width = 0
        if d_height > 0:
            d_width = len(depth_image[0])
        for obj_id, pos in object_list:
            x = int(pos[0] * d_width)
            y = int(pos[1] * d_height)
            distance = depth_image[y, x]
            if distance <= 0:
                continue
            u, _, _ = rs.rs2_deproject_pixel_to_point(color_intrin, (x, y), distance)
            angle = math.asin(u / distance)
            detected_objects.append(ObjectPosition(obj_id, distance, angle))

        if self.object_publisher is not None:
            self.object_publisher.publish(ObjectPositionArray(detected_objects))


if __name__ == '__main__':
    rospy.init_node('object_detection', anonymous=True)
    object_publisher = rospy.Publisher('/detected_objects', ObjectPositionArray, queue_size=10)

    # listener = TestListener(object_publisher, 2500)
    # object_detection = CIBagReader('/home/aimotion/TurtleBot/record3.bag', [listener])
    object_detection = ObjectDetection(object_publisher=object_publisher)
    
    object_detection.read()