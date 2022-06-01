import argparse
import math
import cv2
import numpy as np
import torch
from torchvision import transforms as T
from torchvision.utils import draw_segmentation_masks

import pyrealsense2 as rs

from d435i import RealsenseCamera
from models import load_pretrained_model


class PreProcess(object):
    def __init__(self):
        self.operations = T.Compose([T.ToTensor(), T.Resize((400, 400)),
                                     T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])

    def __call__(self, image):
        image = self.operations(image)
        return image.unsqueeze(0)


class InvertNormalization(object):
    def __init__(self):
        self.invert = T.Normalize(mean=[-0.485 / 0.229, -0.456 / 0.224, -0.406 / 0.225],
                                  std=[1 / 0.229, 1 / 0.224, 1 / 0.225])

    def __call__(self, image):
        return self.invert(image)


def inference(model, image: torch.Tensor) -> torch.Tensor:
    model.eval()
    return model(image.unsqueeze_(0))['out']


def extract_objects(mask: torch.Tensor, obj_classes):
    """
    Takes a Segmentation Mask as Input and detects objects in it.
    Explicitly named 'Iron Man', 'Hulk', 'Captain America', 'Turtlebot' and 'Obstacles'.
    :return: obj_label, Bounding Box
    """
    obj_label, bboxes = list(), list()
    for i, obj in enumerate(obj_classes):
        if i in [0,1,2,4]:
            if torch.max(mask[i]) == 1:
                contour, _ = cv2.findContours(mask[i].numpy().astype('uint8'), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # hull = [cv2.convexHull(c) for c in contour]
                obj_label.append(i)
                bboxes.append(contour)
    return obj_label, bboxes


def calc_angle(intrin, center, depth):
    x_, y_, z_ = rs.rs2_deproject_pixel_to_point(intrin, center, depth)
    u = abs(x_)
    sign_u = 1 if x_ > 0 else -1
    return sign_u * math.degrees(math.asin(u / depth))


def stream_realsense(args):
    rs_cam = RealsenseCamera()
    preprocess = PreProcess()
    model = load_pretrained_model(args)
    while True:
        ret, color_image, depth_image, depth_colormap, intrin = rs_cam.get_frame_stream()

        rgb_tensor = preprocess(color_image)
        heatmap = model(rgb_tensor)['out']
        idx = torch.argmax(heatmap, dim=1, keepdim=True)
        prediction = torch.zeros_like(heatmap).scatter_(1, idx, 1.).squeeze()
        obj_classes, centers = extract_objects(prediction)

        for obj_class, center in zip(obj_classes, centers):
            depth = depth_image[center]
            calc_angle(intrin, center, depth)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    rs_cam.release()


def stream_video(args):
    COLORS = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46)]
    obj_classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacle', 'wall']
    model = load_pretrained_model(args)
    model.eval()
    cap = cv2.VideoCapture(r'data/test_camera_color_image_raw_compressed.mp4')
    if cap.isOpened():
        print("Error opening video file")

    preprocess = PreProcess()
    inv_norm = InvertNormalization()
    # pil = T.ToPILImage()

    n_frame = 0

    while cap.isOpened():
        ret, frame = cap.read()
        n_frame += 1
        if n_frame % 20 == 0:
            if ret:
                frame_tensor = preprocess(frame)
                image = inv_norm(frame_tensor).squeeze()
                output = model(frame_tensor)['out']
                idx = torch.argmax(output, dim=1, keepdim=True)
                preds = torch.zeros_like(output).scatter_(1, idx, 1.).squeeze()
                mask = draw_segmentation_masks(image.to(torch.uint8), preds.to(torch.bool), alpha=0.4, colors=COLORS)

                # scale = (frame.shape[0]/400, frame.shape[1]/400)

                obj_labels, hulls = extract_objects(preds, obj_classes)

                frame = cv2.resize(frame, (400, 400))
                for obj, hull in zip(obj_labels, hulls):
                    # print(obj_classes[obj], hull)
                    cv2.drawContours(frame, hull, -1, COLORS[obj], 3)

                cv2.namedWindow('Frame', cv2.WINDOW_KEEPRATIO)
                cv2.namedWindow('Mask', cv2.WINDOW_KEEPRATIO)
                cv2.imshow('Frame', frame)
                cv2.imshow('Mask', mask.permute(1, 2, 0).numpy())
                cv2.resizeWindow('Frame', 600, 600)
                cv2.resizeWindow('Mask', 600, 600)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    cap.release()


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--weights_dir', type=str, default='checkpoints/lraspp_25_05_2022-21_24/lraspp.pth')
    parser.add_argument('--arch', type=str, default='lraspp', choices=['deeplab', 'lraspp'])
    args = parser.parse_args()
    stream_video(args)


if __name__ == '__main__':
    main()
