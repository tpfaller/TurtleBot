import argparse
import math
import time
import cv2
import json
import numpy as np
import torch
from torchvision import transforms as T
import torchvision.transforms.functional as tf
from torchvision.utils import draw_segmentation_masks

import pyrealsense2 as rs

from d435i import RealsenseCamera
from models import load_pretrained_model
from augmentations import PreProcess, InvertNormalization
import utils


def extract_objects(mask: torch.Tensor, obj_classes, args):
    """
    Takes a Segmentation Mask as Input and detects objects in it.
    Explicitly named 'Iron Man', 'Hulk', 'Captain America', 'Turtlebot' and 'Obstacles'.
    :return: obj_label, Bounding Box
    """
    if args.mode == 'turtlebot':
        objects_ids = [0,1,2,4]
    elif args.mode == 'topdown':
        objects_ids = [0,1,2,4,5,6] 
    obj_label, bboxes = list(), list()
    for i, obj in enumerate(obj_classes):
        if i in objects_ids:
            if torch.max(mask[i]) == 1:
                contour, _ = cv2.findContours(mask[i].numpy().astype('uint8'), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # hull = [cv2.convexHull(c) for c in contour]
                obj_label.append(i)
                bboxes.append(contour)

    return obj_label, bboxes

def extract_objects_new(mask: torch.Tensor, obj_classes, args):
    """
    Takes a Segmentation Mask as Input and detects objects in it.
    Explicitly named 'Iron Man', 'Hulk', 'Captain America', 'Turtlebot' and 'Obstacles'.
    :return: obj_label, Bounding Box
    """
    if args.mode == 'turtlebot':
        objects_ids = [0, 1, 2]
        objects_num = [1, 1, 1]
    elif args.mode == 'topdown':
        objects_ids = [0, 1, 2, 3, 4, 6]
        objects_num = [1, 1, 1, 1, 4, 1]
    obj_label, bboxes = list(), list()
    for obj, num in zip(objects_ids, objects_num):
        # Check if there are pixels from class obj
        if torch.max(mask[obj]) == 1:
            # Find contours in Mask
            contour, _ = cv2.findContours(mask[obj].numpy().astype('uint8'), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Calculate Area per contour object and sort by area
            contour_and_area = [(c, cv2.contourArea(c)) for c in contour]
            contour_and_area.sort(key=lambda x: x[1], reverse=True)

            for index in range(num):
                # Get the given amount of objects, Append None if object detection failed
                try:
                    hull, area = contour_and_area.pop(0)
                except IndexError:
                    hull, area = None, 0
                obj_label.append(obj)
                bboxes.append(hull)

    return obj_label, [bboxes]


def extract_figures(mask: torch.Tensor):
    """
    Takes a Segmentation Mask as Input and detects objects in it.
    Explicitly named 'Iron Man', 'Hulk' and 'Captain America'.
    :return: obj_label, Bounding Box
    """
    obj_label, bboxes, centers = list(), list(), list()
    for i in [0,1,2]:
        if torch.max(mask[i]) == 1:
            contour, _ = cv2.findContours(mask[i].numpy().astype('uint8'), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour = [(c, cv2.contourArea(c)) for c in contour]
            contour.sort(key=lambda x: x[1], reverse=True)
            if contour[0][1] > 300:
                hull = contour[0][0]
                M = cv2.moments(hull)
                m10, m00, m01 = M["m10"], M["m00"]+1e-6, M["m01"]
                centers.append((int(m10 / m00), int(m01 / m00)))
                obj_label.append(i)
                bboxes.append(hull)
    
    return obj_label, bboxes, centers


def calc_angle(intrin, center, depth):
    x, y, z = rs.rs2_deproject_pixel_to_point(intrin, center, depth)
    u = abs(x)
    sign_u = 1 if x > 0 else -1
    return sign_u * math.degrees(math.asin(u / depth))


def rescale(args, center, preprocess):
    x = int(center[0] * args.width / preprocess.width)
    y = int(center[1] * args.height / preprocess.height)
    return x, y


def stream_realsense(args):
    # Initialise Realsense-Camera
    rs_cam = RealsenseCamera(args.width, height=args.height)
    # Initialise Preprocessing and Model
    preprocess = PreProcess()
    model = load_pretrained_model(args)
    model.eval()

    while True:
        # Get Frames from Camera
        ret, rgb_frame, depth_frame, depth_colormap, intrin = rs_cam.get_frame_stream()
        # Preprocess RGB-Frame
        rgb_tensor = preprocess(rgb_frame)

        # Make Prediction
        heatmap = model(rgb_tensor)['out']
        idx = torch.argmax(heatmap, dim=1, keepdim=True)
        prediction = torch.zeros_like(heatmap).scatter_(1, idx, 1.).squeeze()

        # Extract detected Objects from Prediction
        obj_labels, bboxes, t_centers = extract_figures(prediction)

        # Get Depth and Angle per Object
        center_l, depth_l, angle_l = list(), list(), list()
        for obj, boxes, center in zip(obj_labels, bboxes, t_centers):
            x, y = rescale(args, center, preprocess)
            depth = depth_frame[y, x]
            depth_l.append(depth)
            angle_l.append(calc_angle(intrin, (x, y), depth))
            center_l.append((x, y))

        # Information for Path-planning
        info = [*zip(obj_labels, center_l, depth_l, angle_l)]
    # rs_cam.release()


def get_corners(hulls, obj):
    ''' input: hulls of objects
    
        output: list of all four corners and the midpoint'''

    positions = []
    x_list = []
    y_list = []
    for hull in hulls:
        x_list.append(hull[0][0])
        y_list.append(hull[0][1])
    if obj != 3:
            # print("LEN", len(np.where(x_list == min(x_list))[0])-1)
            # print("//2", (len(np.where(x_list == min(x_list))[0])-1)//2)

            # print(np.where(x_list == min(x_list))[0])
            # print(int(x_list[np.where(x_list == min(x_list))[0][(len(np.where(x_list == min(x_list))[0])-1)//2]]))
            # print(int(x_list[np.where(x_list == min(x_list))[0][(len(np.where(x_list == min(x_list))[0]))//2]]))


            # print(np.where(x_list == max(x_list))[0])
            # print()
            # print(np.where(y_list == min(y_list))[0])
            # print()
            # print(np.where(y_list == max(y_list))[0])
            # print()
            # positions.append([  [int(x_list[(len(np.where(x_list == min(x_list))[0])-1)//2]), int(y_list[(len(np.where(x_list == min(x_list))[0])-1)//2])],\
            #                     [int(x_list[(len(np.where(x_list == max(x_list))[0])-1)//2]), int(y_list[(len(np.where(x_list == max(x_list))[0])-1)//2])],\
            #                     [int(x_list[(len(np.where(y_list == min(y_list))[0])-1)//2]), int(y_list[(len(np.where(y_list == min(y_list))[0])-1)//2])],\
            #                     [int(x_list[(len(np.where(y_list == max(y_list))[0])-1)//2]), int(y_list[(len(np.where(y_list == max(y_list))[0])-1)//2])],\
            #                     [int(max(x_list)-((max(x_list)-min(x_list))//2)), int(max(y_list)-((max(y_list)-min(y_list))//2))]])
        positions.append([  [int(x_list[np.where(x_list == min(x_list))[0][(len(np.where(x_list == min(x_list))[0]))//2]]), int(y_list[np.where(x_list == min(x_list))[0][(len(np.where(x_list == min(x_list))[0]))//2]])],\
					            [int(x_list[np.where(x_list == max(x_list))[0][(len(np.where(x_list == max(x_list))[0]))//2]]), int(y_list[np.where(x_list == max(x_list))[0][(len(np.where(x_list == max(x_list))[0]))//2]])],\
					            [int(x_list[np.where(y_list == min(y_list))[0][(len(np.where(y_list == min(y_list))[0]))//2]]), int(y_list[np.where(y_list == min(y_list))[0][(len(np.where(y_list == min(y_list))[0]))//2]])],\
					            [int(x_list[np.where(y_list == max(y_list))[0][(len(np.where(y_list == max(y_list))[0]))//2]]), int(y_list[np.where(y_list == max(y_list))[0][(len(np.where(y_list == max(y_list))[0]))//2]])],\
					            [int(max(x_list)-((max(x_list)-min(x_list))//2)), int(max(y_list)-((max(y_list)-min(y_list))//2))]])
    else:
        positions.append([  [int(x_list[np.where(x_list == min(x_list))[0][0]]), int(y_list[np.where(y_list == min(y_list))[0][0]])],\
                                [int(x_list[np.where(x_list == min(x_list))[0][0]]), int(y_list[np.where(y_list == max(y_list))[0][0]])],\
                                [int(x_list[np.where(x_list == max(x_list))[0][0]]), int(y_list[np.where(y_list == max(y_list))[0][0]])],\
                                [int(x_list[np.where(x_list == max(x_list))[0][0]]), int(y_list[np.where(y_list == min(y_list))[0][0]])],\
                                [int(max(x_list)-((max(x_list)-min(x_list))//2)), int(max(y_list)-((max(y_list)-min(y_list))//2))]])
    return positions

def positions_to_json(obj_classes, obj_labels, hulls):
    ''' output: json with all objects and their positions and midpoints

        syntax dict/json: "object_name": [[corner_1],[corner_2],[corner_3],[corner_4],[mid_point_of_object]'''

    tmp = {}
    tmp["Spielfeld"] = []
    obstacle_count = 0
    for i in range(len(obj_labels)):
        hull = hulls[0][i]
        obj = obj_labels[i]
        figure = obj_classes[obj]
        corners = get_corners(hull, obj)
        if figure == "free_space":
            tmp["Spielfeld"] = [(corners[0][2][0]-corners[0][1][0]),(corners[0][1][1]-corners[0][0][1])]
        elif figure == "obstacles":
            tmp[f"{figure}_{obstacle_count}"] = corners[0]
            obstacle_count += 1      
        else:
            tmp[figure] = corners[0]

    json_tmp = json.dumps(tmp)

    return tmp, json_tmp

def stream_video(args):
    if args.mode == 'turtlebot':
        obj_classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacles', 'wall']
        COLORS = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46)]
    elif args.mode == 'topdown':
        obj_classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacles', 'wall', 'turtlebot', 'background']
        COLORS = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46), (255, 0, 0), (0, 0, 0)]
    model = load_pretrained_model(args)
    model.eval()
    cap = cv2.VideoCapture(args.video)
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

                obj_labels, hulls = extract_objects(preds, obj_classes, args)
                print(positions_to_json(obj_classes, obj_labels, hulls)[0])

                frame = cv2.resize(frame, (400, 400))

                for obj, hull in zip(obj_labels, hulls[0]):
                    # print(obj_classes[obj], hull)
                    cv2.drawContours(frame, hull, -1, COLORS[obj], 3)

                    ## mitte und eckpunkte als rote Punkte anzeigen lassen (debugging)
                    for positions in get_corners(hull, obj):
                        for position in positions:
                            cv2.circle(frame, tuple(position), 2, (0,0,255), 3)

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
    parser.add_argument('--weights_dir', type=str, default='weights/topdown/lraspp_v3.pth')
    parser.add_argument('--video', type=str, default='data/topdown-valid-video.mp4')
    parser.add_argument('--arch', type=str, default='lraspp', choices=['deeplab', 'lraspp'])
    parser.add_argument('--mode', type=str, default='topdown',
                        choices=['turtlebot', 'topdown'])
    args = parser.parse_args()
    stream_video(args)


if __name__ == '__main__':
    main()
