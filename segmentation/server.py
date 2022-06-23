#!/usr/bin/env python3
from argparse import Namespace
from multiprocessing.connection import Listener, Connection
from multiprocessing.reduction import ForkingPickler
import threading
from typing import Dict, List, Optional, Tuple

import torch
from torchvision.utils import draw_segmentation_masks
from torchvision import transforms
from augmentations import InvertNormalization, PreProcess
from inference import extract_figures, extract_objects

from models import load_pretrained_model
import cv2
from pathlib import Path
from dataclasses import dataclass, field
import numpy as np

WEIGHTS_DIR = str(Path(__file__).parent.parent.joinpath('weights/%s/lraspp_%s.pth').absolute())



@dataclass
class ObjectPosition:
    obj_type: 'ObjectType'
    pos: Tuple[float, float]
    dim: Tuple[float, float]
    angle: float
    contour_area: float

    def get_area(self) -> float:
        return self.contour_area
        # return self.dim[0] * self.dim[1]

    def has_min_area(self) -> bool:
        return self.obj_type.min_area is None or self.get_area() >= self.obj_type.min_area

    def as_tuple(self, field_x: float, field_y: float, field_width: float, field_height: float) -> Tuple[str, Tuple[float, float], Tuple[float, float], float]:
        if self.obj_type.fixed_dim is not None:
            self.dim = self.obj_type.fixed_dim
            self.angle = 0
        self.pos = ((self.pos[0] - field_x) / field_width,
                    (self.pos[1] - field_y) / field_height)
        return (self.obj_type.obj_id, self.pos, self.dim, self.angle)


@dataclass
class ObjectType:
    obj_id: str
    max_objects: Optional[int] = None
    fixed_dim: Optional[Tuple[float, float]] = None
    min_area: Optional[float] = None
    priority: int = 0
    min_distance: Dict[str, float] = field(default_factory=dict)

    def filter_positions(self, positions: List[ObjectPosition], previous_positions: List[ObjectPosition]) -> List[ObjectPosition]:
        if self.max_objects is not None and self.max_objects < len(positions):
            
            filtered_positions = list(positions)
            
            for pos in previous_positions:
                min_distance = self.min_distance.get(pos.obj_type.obj_id)
                if min_distance is not None:
                    for obj_pos in positions:
                        if np.linalg.norm(np.array(obj_pos.pos) - np.array(pos.pos)) < min_distance:
                            filtered_positions.remove(obj_pos)
            
            obj_count = min(len(filtered_positions), self.max_objects)
            
            filtered_positions.sort(key=ObjectPosition.get_area, reverse=True)
            return filtered_positions[:obj_count]
        else:
            return list(positions)


OBJ_TYPES = [
    ObjectType('iron_man', 1, (0.07, 0.07), priority=1, min_distance={'turtlebot': 0.1}),
    ObjectType('captain_america', 1, (0.07, 0.07), min_distance={'iron_man': 0.1}),
    ObjectType('hulk', 1, (0.07, 0.07)),
    ObjectType('free_space'),
    ObjectType('obstacles', min_area=0.005),
    ObjectType('wall'),
    ObjectType('turtlebot', 1, (0.1, 0.1), priority=2),
    ObjectType('background')
]

OBJ_CLASSES = [obj_type.obj_id for obj_type in OBJ_TYPES]

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


def handle_client(conn: Connection):

    mode = conn.recv()

    
    obj_priority = [obj_id for obj_id, _ in sorted(enumerate(OBJ_TYPES), key=lambda entry: entry[1].priority, reverse=True)]

    if mode == 'turtlebot':
        weight_version = 'v2'
    elif mode == 'topdown':
        weight_version = 'v3'
    else:
        print('Unknown mode: %s' % str(mode))
        conn.close()

    print("Initializing mode '%s'" % mode)

    args = Namespace(
        mode=mode,
        weights_dir=WEIGHTS_DIR % (mode, weight_version),
        arch='lraspp',
        obj_classes=OBJ_CLASSES
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
            
            # cv2.imshow('test', image)
            # cv2.waitKey(0)

            # Preprocess RGB-Frame
            rgb_tensor = preprocess(image)

            # Make Prediction
            heatmap = model(rgb_tensor)['out']
            idx = torch.argmax(heatmap, dim=1, keepdim=True)
            prediction = torch.zeros_like(heatmap).scatter_(1, idx, 1.).squeeze()

            # Extract detected Objects from Prediction
            if mode == 'turtlebot':
                obj_labels, bboxes, t_centers = extract_figures(prediction)
                # COLORS = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46), (255, 0, 0), (0, 0, 0)]
                # image = InvertNormalization()(rgb_tensor)
                # mask = draw_segmentation_masks(image.squeeze().to(torch.uint8), prediction.to(torch.bool), alpha=0.4, colors=COLORS)
                # fr = mask.permute(1,2,0).numpy()
                # print(image.size(), rgb_tensor.size())
                # cv2.imshow('test', fr)
                # if cv2.waitKey(25) & 0xFF == ord('q'):
                #     cv2.destroyAllWindows()
                objects = [(args.obj_classes[obj], normalize(pos, preprocess))
                            for (obj, pos) in zip(obj_labels, t_centers)]
            else:
                obj_labels, hulls = extract_objects(prediction, OBJ_CLASSES, args)
                # COLORS = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46), (255, 0, 0), (0, 0, 0)]
                # image = InvertNormalization()(rgb_tensor)
                # mask = draw_segmentation_masks(image.squeeze().to(torch.uint8), prediction.to(torch.bool), alpha=0.4, colors=COLORS)
                # fr = mask.permute(1,2,0).numpy()
                # print(image.size(), rgb_tensor.size())
                # cv2.imshow('test', fr)
                # cv2.waitKey(0)
                objects = []

                field_x = 0
                field_y = 0
                field_width = 1
                field_height = 1
                
                possible_positions: Dict[int, List[ObjectPosition]] = {}

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
                        obj_type = OBJ_TYPES[obj_id]

                        positions = []
                        for hull in hull_list:
                            rect = get_normalized_rotated_rect(hull, preprocess)
                            pos = ObjectPosition(obj_type, *rect, cv2.contourArea(hull))
                            if not pos.has_min_area():
                                continue
                            positions.append(pos)

                        if len(positions) > 0:
                            possible_positions[obj_id] = positions

                for obj_id in obj_priority:

                    positions = possible_positions.get(obj_id)
                    if positions is None:
                        continue

                    obj_type = OBJ_TYPES[obj_id]
                    filtered_objects = obj_type.filter_positions(positions, objects)

                    for obj in filtered_objects:
                        objects.append(obj)
                
                objects = [obj.as_tuple(field_x, field_y, field_width, field_height) for obj in objects]

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
