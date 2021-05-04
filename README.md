ArUco based Quad-copter auto-landing system
=============
*single asterisks*This project is vision base system for Quad-copter auto-laning system
*single asterisks*H/W system will be updated
*single asterisks*Now code is only in vision system on python

REQUIREMENTS
-------------
*single asterisks*This project uses Intel realsense camera L515 
*single asterisks*openCV + ArUco marker detection system

USAGE
-------------
You should use 8x6 chess aruco plate   
(by marker_generation/generate_ChAruco.py)

    python calibration.py -i {bag file name} -c {No. of captures}

=> You can get CameraCalibration.pckl

You should use 3x4 chess aruco plate    
(by marker_generation/generate_arucoGrid.py)

    python main.py

REFERENCES
-------------
<https://github.com/ddelago/Aruco-Marker-Calibration-and-Pose-Estimation>