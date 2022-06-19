#!/usr/bin/env python3
from argparse import Namespace
from multiprocessing.connection import Listener, Connection
from multiprocessing.reduction import ForkingPickler
import threading

import torch
from augmentations import PreProcess
from inference import extract_figures, extract_objects

from models import load_pretrained_model
import cv2
import numpy as np

WEIGHTS_DIR = '/home/aimotion/TurtleBot/TurtleBot/weights/%s/lraspp_30.pth'

def get_normalized_bounding_rect(hull, width, height):
    pos, dim, angle = cv2.minAreaRect(hull)
    return ((pos[0] / width, pos[1] / height),
            (dim[0] / width, dim[1] / height),
            angle)


def get_area(rect):
    dim = rect[1]
    return dim[0] * dim[1]


def handle_client(conn: Connection):

    mode = conn.recv()

    max_object_counts = [1, 1, 1, None, None, None, 1, None]
    fixed_dimensions = [(0.07, 0.07), (0.07, 0.07), (0.07, 0.07), None, None, None, (0.1, 0.1), None]

    if mode == 'turtlebot':
        obj_classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacles', 'wall']
    elif mode == 'topdown':
        obj_classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacles', 'wall', 'turtlebot', 'background']
    else:
        print('Unknown mode: %s' % str(mode))
        conn.close()

    print("Initializing mode '%s'" % mode)

    args = Namespace(
        mode=mode,
        weights_dir=WEIGHTS_DIR % mode,
        arch='lraspp',
        obj_classes=obj_classes
    )

    model = load_pretrained_model(args)
    model.eval()
    preprocess = PreProcess()
    try:
        while True:
            # image = conn.recv()
            # Workaround for python2 compatibility
            conn._check_closed()
            conn._check_readable()
            buf = conn._recv_bytes()
            image = ForkingPickler.loads(buf.getbuffer(), encoding='bytes')
            
            # Preprocess RGB-Frame
            rgb_tensor = preprocess(image)

            # Make Prediction
            heatmap = model(rgb_tensor)['out']
            idx = torch.argmax(heatmap, dim=1, keepdim=True)
            prediction = torch.zeros_like(heatmap).scatter_(1, idx, 1.).squeeze()

            # Extract detected Objects from Prediction
            if mode == 'turtlebot':
                obj_labels, bboxes, t_centers = extract_figures(prediction)
                objects = [(args.obj_classes[obj], (pos[0] / preprocess.width, pos[1] / preprocess.height))
                            for (obj, pos) in zip(obj_labels, t_centers)]
            else:
                obj_labels, hulls = extract_objects(prediction, obj_classes, args)
                objects = []

                for obj_id, hull_list in zip(obj_labels, hulls):

                    max_object_count = max_object_counts[obj_id]
                    fixed_dim = fixed_dimensions[obj_id]

                    bounding_rects = []
                    for hull in hull_list:
                        rect = get_normalized_bounding_rect(hull, preprocess.width, preprocess.height)
                        bounding_rects.append(rect)

                    object_count = len(bounding_rects)

                    if max_object_count is not None and object_count > max_object_count:
                        bounding_rects.sort(key=get_area, reverse=True)
                        object_count = max_object_count

                    for i in range(object_count):
                        pos, dim, angle = bounding_rects[i]
                        if fixed_dim is not None:
                            dim = fixed_dim
                            angle = 0
                        obj_data = (obj_classes[obj_id], pos, dim, angle)
                        objects.append(obj_data)

            # conn.send(objects)
            # Workaround for python2 compatibility
            conn._check_closed()
            conn._check_writable()
            conn._send_bytes(ForkingPickler.dumps(objects, protocol=2))

    finally:
        conn.close()

if __name__ == '__main__':
    address = ('localhost', 6000)
    listener = Listener(address)
    try:
        while True:
            conn = listener.accept()
            client_thead = threading.Thread(target=handle_client, args=(conn,), daemon=True)
            client_thead.start()
    finally:
        listener.close()
