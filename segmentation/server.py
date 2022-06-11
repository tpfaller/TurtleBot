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

WEIGHTS_DIR = '/home/aimotion/TurtleBot/TurtleBot/weights/%s/lraspp_30.pth'

def handle_client(conn: Connection):

    mode = conn.recv()

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
            # Workaround for python2 comppatibility
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
            obj_labels, bboxes, t_centers = extract_figures(prediction)

            objects = [(args.obj_classes[obj], (pos[0] / preprocess.width, pos[1] / preprocess.height))
                        for (obj, pos) in zip(obj_labels, t_centers)]

            # conn.send(objects)
            # Workaround for python2 comppatibility
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
