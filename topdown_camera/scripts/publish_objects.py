#!/usr/bin/env python
from multiprocessing.connection import Client

import cv2
import rospy
from topdown_camera.msg import ObjectPose, ObjectPoseArray

class CameraReader(object):

    def __init__(self, publisher, obj_width = 0.1, obj_height = 0.1):
        self.client = Client(('localhost', 6000))
        self.publisher = publisher
        self.client.send('topdown')
        self.obj_width = obj_width
        self.obj_height = obj_height

    def handle_image(self, np_image):
        self.client.send(np_image)
        object_list = self.client.recv()
        obstacle_id = 0
        detected_objects = []

        for obj_id, pos, dim, rot in object_list:
            pos = (pos[0] - self.obj_width / 2,
                   pos[1] - self.obj_height / 2)

            if obj_id == 'obstacles':
                obj_id = 'obstacle_%d' % obstacle_id
                obstacle_id += 1
            if obj_id == 'hulk':
                obj_id = 'Hulk'
            if obj_id == 'iron_man':
                obj_id = 'Iron_Man'
            if obj_id == 'captain_america':
                obj_id = 'Captain_America'
            if rot > 45:
                rot -= 90
                dim = (dim[1], dim[0])
            detected_objects.append(ObjectPose(obj_id, pos[0], pos[1], dim[0], dim[1], rot))

        self.publisher.publish(ObjectPoseArray(detected_objects))

    def read(self):
        cap = cv2.VideoCapture(0)
        while cap.isOpened():
            ret, frame = cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if ret:
                # cv2.imshow('test', frame)
                # cv2.waitKey(0)
                self.handle_image(frame)


def publish_objects():
    pub = rospy.Publisher('/game_objects', ObjectPoseArray, queue_size=10)
    rospy.init_node('topdown_camera', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     pub.publish(ObjectPoseArray([
    #         ObjectPose('turtlebot', 0.1, 0.1, 0.1, 0.1, 0),
    #         ObjectPose('obstacle_0', 0.3, 0.3, 0.1, 0.2, 0),
    #         ObjectPose('obstacle_1', 0.1, 0.7, 0.3, 0.1, 0),
    #         ObjectPose('obstacle_2', 0.7, 0.7, 0.1, 0.1, 0),
    #         ObjectPose('Captain_America', 0.5, 0.5, 0.1, 0.1, 0),
    #         ObjectPose('Hulk', 0.1, 0.5, 0.1, 0.1, 0),
    #         ObjectPose('Iron_Man', 0.8, 0.2, 0.1, 0.1, 0)]))
    #     rate.sleep()
    reader = CameraReader(pub)
    reader.read()

if __name__ == '__main__':
    try:
        publish_objects()
    except rospy.ROSInterruptException:
        pass
