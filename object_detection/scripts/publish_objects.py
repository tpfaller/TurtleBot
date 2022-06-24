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
    'iron_man': (0, 0, 255),
    'hulk': (0, 255, 0),
    'captain_america': (255, 0, 0),
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
        np_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
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

    def __init__(self, server_address = ('localhost', 6000), object_publisher = None, width=640, height=480, fps=30, visualize = False):
        super(ObjectDetection, self).__init__(width, height, fps, None, None)
        self.client = Client(server_address)
        self.object_publisher = object_publisher
        self.visualize = visualize
        self.client.send('turtlebot')

    def handle_images(self, color_image, depth_image, color_timestamp, depth_timestamp, depth_intrin):
        
        super(ObjectDetection, self).handle_images(color_image, depth_image, color_timestamp, depth_timestamp, depth_intrin)

        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        self.client.send(color_image)
        object_list = self.client.recv()

        detected_objects = []

        d_height = len(depth_image)
        d_width = 0
        if d_height > 0:
            d_width = len(depth_image[0])

        c_height = len(color_image)
        c_width = 0
        if c_height > 0:
            c_width = len(color_image[0])

        frame = color_image
        
        for obj_id, pos in object_list:
            x = int(pos[0] * d_width)
            y = int(pos[1] * d_height)
            distance = depth_image[y, x]

            if distance <= 0:
                continue
            if self.visualize:
                color = COLORS.get(obj_id, (0, 255, 255))
                frame = cv2.circle(frame, (int(pos[0] * c_width), int(pos[1] * c_height)), 5, color, thickness=-1)

            u, _, _ = rs.rs2_deproject_pixel_to_point(depth_intrin, (x, y), distance)
            angle = math.asin(u / distance)
            detected_objects.append(ObjectPosition(obj_id, distance, angle))

        if self.object_publisher is not None:
            self.object_publisher.publish(ObjectPositionArray(detected_objects))
        
        if self.visualize:
            cv2.imshow('objects', frame)
        
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('object_detection', anonymous=True)
    object_publisher = rospy.Publisher('/detected_objects', ObjectPositionArray, queue_size=10)

    # listener = TestListener(object_publisher, 500)
    # object_detection = CIBagReader('/home/manuel/TurtleBot/2022-04-26-12-45-00.bag', [listener])
    object_detection = ObjectDetection(object_publisher=object_publisher, visualize=False)

    object_detection.read()