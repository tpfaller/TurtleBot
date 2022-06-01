import argparse
import cv2
import numpy as np
import pyrealsense2 as rs


class RealsenseCamera:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30, file_path: str = None):
        # Configure depth and color streams
        print("Loading Intel Realsense Camera")
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        # Start streaming
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame_stream(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            print(
                "Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
            return False, None, None

        color_profile = color_frame.get_profile()
        cvsprofile = rs.video_stream_profile(color_profile)
        color_intrin = cvsprofile.get_intrinsics()

        # Apply filter to fill the Holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)
        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)

        # Create colormap to show the depth of the Objects
        colorizer = rs.colorizer()
        depth_colormap = np.asanyarray(colorizer.colorize(filled_depth).get_data())

        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return True, color_image, depth_image, depth_colormap, color_intrin

    def release(self):
        self.pipeline.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--path', type=str, default='data/bagfiles/2022-04-26-12-45-00.bag')
    args = parser.parse_args()
    rs_cam = RealsenseCamera(args.width, args.height, args.fps, args.path)
    while True:
        ret, color_image, depth_image, depth_colormap, intrin = rs_cam.get_frame_stream()
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.10), 2)
        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('Test', cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow('Color Frame', cv2.WINDOW_KEEPRATIO)
        cv2.imshow('Test', depth_colormap)
        cv2.imshow('Color Frame', color_image)
        cv2.resizeWindow('Test', 600, 600)
        cv2.resizeWindow('Color Frame', 600, 600)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    rs_cam.release()


if __name__ == '__main__':
    main()
