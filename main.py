import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import os
import pickle


# def findArucoMarkers(img, markerSize=5, totalMarkers=250, draw=True):
#     imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
#     arucoDict = aruco.Dictionary_get(key)
#     arucoParam = aruco.DetectorParameters_create()
#     bboxs, ids, rejected = aruco.detectMarkers(imgGray, 
#                                                 arucoDict, 
#                                                 parameters=arucoParam)
#     if draw:
#         aruco.drawDetectedMarkers(img, bboxs)
#     return bboxs, ids


def main():

    if not os.path.exists('CameraCalibration.pckl'):
        print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
        exit()
    else:
        f = open('CameraCalibration.pckl', 'rb')
        (cameraMatrix, distCoeffs, _, _) = pickle.load(f)
        f.close()
        if cameraMatrix is None or distCoeffs is None:
            print("Calibration issue. Remove CameraCalibration.pckl and recalibrate your camera with calibration_ChAruco.py.")
            exit()

    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        print("get L500 product line && stream color size 1280x720")
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 1  # 1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Constant parameters used in Aruco methods
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)

    # Create grid board object we're using in our stream
    board = aruco.GridBoard_create(
            markersX=1,
            markersY=1,
            markerLength=0.09,
            markerSeparation=0.01,
            dictionary=ARUCO_DICT)

    # Create vectors we'll be using for rotations and translations for postures
    rotation_vectors, translation_vectors = None, None
    axis = np.float32([[-.5,-.5,0], [-.5,.5,0], [.5,.5,0], [.5,-.5,0],
                    [-.5,-.5,1],[-.5,.5,1],[.5,.5,1],[.5,-.5,1] ])

    # Streaming loop
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            # aligned_depth_frame is a 640x480 depth image
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            # Detect Aruco markers
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    
            # Refine detected markers
            # Eliminates markers not part of our board, adds missing markers to the board
            corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
                    image = gray,
                    board = board,
                    detectedCorners = corners,
                    detectedIds = ids,
                    rejectedCorners = rejectedImgPoints,
                    cameraMatrix = cameraMatrix,
                    distCoeffs = distCoeffs)   

            #corners, ids = findArucoMarkers(color_image)

            # Outline all of the markers detected in our image
            # Uncomment below to show ids as well
            # ProjectImage = aruco.drawDetectedMarkers(ProjectImage, corners, ids, borderColor=(0, 0, 255))
            color_image = aruco.drawDetectedMarkers(color_image, corners, borderColor=(0, 0, 255))

            if ids is not None and len(ids) > 0:
                # Estimate the posture per each Aruco marker
                rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(corners, 1, cameraMatrix, distCoeffs)
                
                for rvec, tvec in zip(rotation_vectors, translation_vectors):
                    if len(sys.argv) == 2 and sys.argv[1] == 'cube':
                        try:
                            imgpts, jac = cv2.projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs)
                            color_image = drawCube(color_image, corners, imgpts)
                        except:
                            continue
                    else:    
                        color_image = aruco.drawAxis(color_image, cameraMatrix, distCoeffs, rvec, tvec, 1)

            cv2.imshow('test frame', color_image)

            # Press esc or 'q' to close the image window
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()