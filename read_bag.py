import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os.path

parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                Remember to change the stream fps and format to match the recorded.")
parser.add_argument("-i", "--input", type=str, help="Path to the bag file")
args = parser.parse_args()
if not args.input:
    print("No input paramater have been given.")
    print("For help type --help")
    exit()
if os.path.splitext(args.input)[1] != ".bag":
    print("The given file is not of correct file format.")
    print("Only .bag files are accepted")
    exit()
try:
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, args.input)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)
    cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)
    colorizer = rs.colorizer()

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_frame = np.asanyarray(color_frame.get_data())

        cv2.imshow("Depth Stream", color_frame)
        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pass