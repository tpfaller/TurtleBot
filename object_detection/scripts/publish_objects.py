#!/usr/bin/env python
from camera_reader import CIBagReader, ImageListener, RealsenseReader
from topics import CITopic
import rospy
from object_detection.msg import ObjectPosition, ObjectPositionArray
from multiprocessing.connection import Client

# region: testing

class TestListener(ImageListener):

    def __init__(self, object_publisher):
        super(TestListener, self).__init__(CITopic.color_img.value)
        self.client = Client(('localhost', 6000))
        self.object_publisher = object_publisher

    def handle_img(self, np_image, timestamp):
        super(TestListener, self).handle_img(np_image, timestamp)
        self.client.send(np_image)
        object_dict = self.client.recv()
        detected_objects = []
        for obj_id, pos in object_dict.items():
            distance = 10 # TODO: calculate distance
            angle = 0 # TODO: calculate angle
            detected_objects.append(ObjectPosition(obj_id, distance, angle))

        if self.object_publisher is not None:
            self.object_publisher.publish(ObjectPositionArray(detected_objects))

# endregion

class ObjectDetection(RealsenseReader):

    def __init__(self, server_address = ('localhost', 6000), object_publisher = None, width=640, height=480, fps=30):
        super(ObjectDetection, self).__init__(width, height, fps, None, None)
        self.client = Client(server_address)
        self.object_publisher = object_publisher

    def handle_images(self, color_image, depth_image, color_timestamp, depth_timestamp, depth_colormap, color_intrin):
        super(ObjectDetection, self).handle_images(color_image, depth_image, color_timestamp, depth_timestamp, depth_colormap, color_intrin)
        self.client.send(color_image)
        object_dict = self.client.recv()
        detected_objects = []
        for obj_id, pos in object_dict.items():
            distance = 10 # TODO: calculate distance
            angle = 0 # TODO: calculate angle
            detected_objects.append(ObjectPosition(obj_id, distance, angle))

        if self.object_publisher is not None:
            self.object_publisher.publish(ObjectPositionArray(detected_objects))


if __name__ == '__main__':
    rospy.init_node('object_detection', anonymous=True)
    object_publisher = rospy.Publisher('/detected_objects', ObjectPositionArray, queue_size=10)

    # listener = TestListener(object_publisher)
    # object_detection = CIBagReader('/home/aimotion/TurtleBot/record3.bag', [listener])
    object_detection = ObjectDetection(object_publisher=object_publisher)
    
    object_detection.read()