import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import os
import pickle
import math

def drawCube(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)

    # draw ground floor in green
    # img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)

    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)

    return img

def ra2de(ra):
    return ra*180/math.pi

def Rmat2euler(rmat):
    [[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]] = rmat
    x = math.atan2(r32, r33)
    y = math.atan2(-r31, math.sqrt(pow(r32,2)+pow(r33,2)))
    z = math.atan2(r21,r11)
    #print(ra2de(x), ra2de(y), ra2de(z))
    print("Angle_X = {0:+.2f}, Angle_Y = {1:+.2f}, Angle_Z = {2:+.2f}".format(ra2de(x), ra2de(y), ra2de(z)))
    return x,y,z

def imu2Rgb(imu):
    x_de, y_de, z_de = imu
    x = x_de*math.pi/180
    y = y_de*math.pi/180
    z = z_de*math.pi/180
    
    #ZYX rotation
    r11 = math.cos(y)*math.cos(z)
    r12 = math.sin(x)*math.sin(y)*math.cos(z)-math.sin(x)*math.cos(z)
    r13 = math.cos(x)*math.sin(y)*math.cos(z)+math.sin(x)*math.sin(z)
    r21 = math.cos(y)*math.sin(z)
    r22 = math.sin(x)*math.sin(y)*math.sin(z)+math.cos(x)*math.cos(z)
    r23 = math.cos(x)*math.sin(y)*math.sin(z)-math.sin(x)*math.cos(z)
    r31 = -math.sin(y)
    r32 = math.sin(x)*math.cos(y)
    r33 = math.cos(x)*math.cos(y)
    
    rgb = np.array([[r11, r12, r13], [r21, r22, r23],[r31, r32, r33]])
    return rgb

# def getleg(rmat):
#     [[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]] = rmat
#     A = [[1,0,-r13],[0,1,-r23],[0,0,-r33]]
#     B = [[r11,r12],[r21,r22],[r31,r32]]
#     C = np.transpose([1,1])
#     leg_axis1 = np.transpose([[1,1],[-1,1],[1,-1],[-1,-1]])
#     leg_axis2 = [[1,1],[-1,1],[1,-1],[-1,-1]]
#     A_inv = np.linalg.inv(A)
#     L = 95
#     #print(np.dot(A_inv,np.dot(B,leg_axis1))[2]*95)
#     #print(np.transpose(np.dot(A_inv,np.dot(B,leg_axis))*95))
#     #print(np.dot(np.dot(A_inv,B),C)[2]*95)
#     print(r31, r32, r33)
#     #print((-L/r33)*np.dot(leg_axis2, [[r31],[r32]]))

#     angle_x,angle_y,angle_z = Rmat2euler(rmat)
#     #z1 = (math.sin(angle_y)-math.sin(angle_x)*math.cos(angle_y))/(math.cos(angle_y)*math.cos(angle_y))*L
#     #print(z1)
    
def getleg(Rct, imu):
    Rgb = imu2Rgb(imu)
    Rbc = np.array([[1,0,0], [0,-1,0], [0,0,-1]])  # Axis-x 180 degree rotation
    
    # Total rotation matrix . from ground to target
    Rgt = np.dot(np.dot(Rgb,Rbc),Rct)
    x, y, z = Rmat2euler(Rgt)
    # Rbt = np.dot(Rbc,Rct)
    # x, y, z = Rmat2euler(Rbt)

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

    # # Create grid board object we're using in our stream
    # board = aruco.GridBoard_create(
    #         markersX=1,
    #         markersY=1,
    #         markerLength=0.09,
    #         markerSeparation=0.01,
    #         dictionary=ARUCO_DICT)

    # Create vectors we'll be using for rotations and translations for postures
    rotation_vectors, translation_vectors = None, None
    # axis = np.float32([[-.5,-.5,0], [-.5,.5,0], [.5,.5,0], [.5,-.5,0],
    #                 [-.5,-.5,1],[-.5,.5,1],[.5,.5,1],[.5,-.5,1] ])
    axis = np.float32([[-1,-1,0], [-1,1,0], [1,1,0], [1,-1,0],
                    [-1,-1,1],[-1,1,1],[1,1,1],[1,-1,1] ])                

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
            # corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
            #         image = gray,
            #         board = board,
            #         detectedCorners = corners,
            #         detectedIds = ids,
            #         rejectedCorners = rejectedImgPoints,
            #         cameraMatrix = cameraMatrix,
            #         distCoeffs = distCoeffs)   

            #corners, ids = findArucoMarkers(color_image)

            # Outline all of the markers detected in our image
            # Uncomment below to show ids as well
            color_image = aruco.drawDetectedMarkers(color_image, corners, borderColor=(0, 0, 255))
            
            imu_angle = [0,0,0] ## imu value not be changed

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
                        Rmat, jacobian = cv2.Rodrigues(rvec)
                        getleg(Rmat, imu_angle)
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