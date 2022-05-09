#!/usr/bin/env python
import math
from camera_reader import ImageListener
import cv2


class VideoWriter(ImageListener):

    def __init__(self, topic, filename, fourcc = cv2.VideoWriter_fourcc(*'mp4v'), fps = 30):
        super(VideoWriter, self).__init__(topic)
        self.filename = filename
        self.fourcc = fourcc
        self.fps = fps
        self.start_time = None
        self.frame_count = 0
        self.writer = None

    def handle_img(self, np_image, timestamp):
        super(VideoWriter, self).handle_img(np_image, timestamp)
        if self.writer is None:
            height = len(np_image)
            width = 0
            if height > 0:
                width = len(np_image[0])
            print('Creating video writer with dimensions %d x %d for %s' % (width, height, self.filename))
            self.writer = cv2.VideoWriter(self.filename, self.fourcc, self.fps, (width, height))
            self.start_time = timestamp
        
        length = timestamp - self.start_time
        count = self.fps * length - self.frame_count + 1
        if count < 0:
            count = 0
        else:
            count = math.floor(count)

        for _ in range(int(count)):
            self.writer.write(np_image)
            self.frame_count += 1



