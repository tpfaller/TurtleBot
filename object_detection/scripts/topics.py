#!/usr/bin/env python
from enum import Enum

class CITopic(str, Enum):
    color_img = '/camera/color/image_raw/compressed'
    depth_img = '/camera/depth/image_rect_raw/compressed'
