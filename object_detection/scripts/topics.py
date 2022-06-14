#!/usr/bin/env python
from enum import Enum

class CITopic(str, Enum):
    color_img = '/camera/color/image_raw/compressed'
    depth_img = '/camera/depth/image_rect_raw/compressed'


class CameraTopic(str, Enum):
    depth_img = '/camera/depth/image_rect_raw'
    camera_info_color = '/camera/color/camera_info'
    camera_info_depth = '/camera/depth/camera_info'
