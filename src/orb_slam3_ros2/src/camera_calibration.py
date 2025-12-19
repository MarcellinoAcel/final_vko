import pyrealsense2 as rs
import numpy as np
import cv2 as cv

# Chessboard settings
CHECKERBOARD = (7, 6)
square_size = 0.025  # meters

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

# RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
pipeline.start(config)

print("Press SPACE to capture image, ESC to finish")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, None)

        cv.imshow("RealSense Calibration", img)
        key = cv.waitKey(1)

        if ret:
            corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            cv.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            
            if key == 32:  # SPACE
                objpoints.append(objp)
                imgpoints.append(corners2)
                print(f"Captured {len(objpoints)} images")

        if key == 27:  # ESC
            break

finally:
    pipeline.stop()
    cv.destroyAllWindows()

# Run calibration
mtx = np.zeros((3, 3))
dist = np.zeros((4, 1))
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], mtx, dist)

print("\nCamera matrix:\n", mtx)
print("\nDistortion coefficients:\n", dist)
