#!/usr/bin/env python3
from argparse import Namespace
from multiprocessing.connection import Listener, Connection
from multiprocessing.reduction import ForkingPickler
import threading
from typing import Tuple

import torch
from augmentations import PreProcess
from inference import extract_figures, extract_objects

from models import load_pretrained_model
import cv2
from pathlib import Path
from dataclasses import dataclass
import numpy as np

WEIGHTS_DIR = str(Path(__file__).parent.parent.joinpath('weights/%s/lraspp_%s.pth').absolute())

@dataclass
class ObjectPosition:
    obj_id: str
    pos: Tuple[float, float]
    dim: Tuple[float, float]
    angle: float

    def as_tuple(self):
        return (self.obj_id, self.pos, self.dim, self.angle)

def normalize(pos: Tuple[int, int], preprocess: PreProcess) -> Tuple[float, float]:
    return (pos[0] / preprocess.width, pos[1] / preprocess.height)

def get_normalized_rect(hull, preprocess: PreProcess):
    min_x = min(hull, key=lambda point: point[0][0])[0][0]
    min_y = min(hull, key=lambda point: point[0][1])[0][1]
    max_x = max(hull, key=lambda point: point[0][0])[0][0]
    max_y = max(hull, key=lambda point: point[0][1])[0][1]

    pos = (min_x, min_y)
    dim = (max_x - min_x, max_y - min_y)

    return (normalize(pos, preprocess), normalize(dim, preprocess))


def get_normalized_rotated_rect(hull, preprocess: PreProcess):
    pos, dim, angle = cv2.minAreaRect(hull)
    return (normalize(pos, preprocess), normalize(dim, preprocess), angle)


def get_area(rect):
    dim = rect[1]
    return dim[0] * dim[1]


def handle_client(conn: Connection):

    mode = conn.recv()

    max_object_counts = [1, 1, 1, None, None, None, 1, None]
    fixed_dimensions = [(0.07, 0.07), (0.07, 0.07), (0.07, 0.07), None, None, None, (0.1, 0.1), None]
    min_areas = [None, None, None, None, 0.005, None, None, None]

    if mode == 'turtlebot':
        obj_classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacles', 'wall']
        weight_version = 'v2'
    elif mode == 'topdown':
        obj_classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacles', 'wall', 'turtlebot', 'background']
        weight_version = 'v1'
    else:
        print('Unknown mode: %s' % str(mode))
        conn.close()

    print("Initializing mode '%s'" % mode)

    args = Namespace(
        mode=mode,
        weights_dir=WEIGHTS_DIR % (mode, weight_version),
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
                objects = [(args.obj_classes[obj], normalize(pos, preprocess))
                            for (obj, pos) in zip(obj_labels, t_centers)]
            else:
                obj_labels, hulls = extract_objects(prediction, obj_classes, args)
                objects = []

                field_x = 0
                field_y = 0
                field_width = 1
                field_height = 1

                for obj_id, hull_list in zip(obj_labels, hulls):

                    if obj_id == 5:
                        # Handle walls
                        # min_x, min_y, max_x, max_y
                        borders = [0, 0, 1, 1]

                        for hull in hull_list:
                            pos, dim = get_normalized_rect(hull, preprocess)
                            side = 0
                            if dim[0] > dim[1]:
                                side = 1
                            if pos[side] < 0.5:
                                borders[side] = max(pos[side] + dim[side] - 0.05, 0)
                            else:
                                borders[side + 2] = min(pos[side] + 0.05, 1)
                        
                        field_x = borders[0]
                        field_y = borders[1]
                        field_width = borders[2] - borders[0]
                        field_height = borders[3] - borders[1]
                    else:
                        # Handle game objects
                        max_object_count = max_object_counts[obj_id]
                        fixed_dim = fixed_dimensions[obj_id]
                        min_area = min_areas[obj_id]

                        bounding_rects = []
                        for hull in hull_list:
                            rect = get_normalized_rotated_rect(hull, preprocess)
                            if min_area is not None and get_area(rect) < min_area:
                                continue
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
                            obj_data = ObjectPosition(obj_classes[obj_id], pos, dim, angle)
                            objects.append(obj_data)

                for object in objects:
                    object.pos = ((object.pos[0] - field_x) / field_width,
                                  (object.pos[1] - field_y) / field_height)

                objects = [obj.as_tuple() for obj in objects]

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
