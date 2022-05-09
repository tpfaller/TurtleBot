#!/usr/bin/env python
from datetime import datetime
from topics import CITopic
from video_writer import VideoWriter
from camera_reader import CIBagReader, CISubscriber
import argparse as ap
import os


if __name__ == '__main__':
    parser = ap.ArgumentParser(
        description='Reads the following topics and saves them to a video files: ' 
                    + ', '.join(t.value for t in CITopic)
    )

    parser.add_argument('-b', '--bag-file', help='Bag file used for input')
    parser.add_argument('-p', '--filename-prefix', default=datetime.now().isoformat(), help='Prefix used for video file names')
    parser.add_argument('-f', '--fps', default=30, help='FPS used for video')
    parser.add_argument('dir', nargs='?', default='.', help='Directory for video files')

    args = parser.parse_args()

    listeners = []

    for topic in CITopic:
        writer = VideoWriter(
            topic=topic.value,
            filename=os.path.join(args.dir, args.filename_prefix + topic.value.replace('/', '_') + '.mp4'),
            fps=args.fps
        )
        listeners.append(writer)

    if args.bag_file is not None:
        reader = CIBagReader(args.bag_file, listeners=listeners)
    else:
        reader = CISubscriber(listeners=listeners)

    reader.read()

