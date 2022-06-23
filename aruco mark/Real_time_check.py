import cv2
import cv2.aruco as aruco
import numpy as np
import os

# CAP_DSHOW : Fast Speed
cap = cv2.VideoCapture(cv2.CAP_DSHOW + 1)
cap.isOpened()

# Cam Setting

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FRAME_COUNT, 60)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Set camMatrix
cam_f = np.array([766.8537, 769.1458])
cam_c = np.array([647.6205, 345.3208])

cam_dist_r = np.array([0.1313, -0.2132])
cam_dist_t = np.array([0, 0])

mtx = np.array([[cam_f[0], 0, cam_c[0]], [0, cam_f[1], cam_c[1]], [0, 0, 1]])
dist = np.hstack([cam_dist_r, cam_dist_t])


def findAruco(img, marker_size=4, total_markers=250, draw=True):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{marker_size}X{marker_size}_{total_markers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)

    # font for displaying text (below)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # check if the ids list is not empty
    # if no check is added the code will crash
    if np.all(ids != None):

        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(bboxs, 0.05, mtx, dist)
        # (rvec-tvec).any() # get rid of that nasty numpy value array error

        print(rvec[0])
        print(tvec[0])
        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(img, mtx, dist, rvec[i], tvec[i], 0.1)

        # draw a square around the markers
        aruco.drawDetectedMarkers(img, bboxs)

        # code to show ids of the marker found
        strg = ''
        for i in range(0, ids.size):
            strg += str(ids[i][0]) + ', '

        cv2.putText(img, "Id: " + strg, (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        # code to show 'No Ids' when no markers are found
        cv2.putText(img, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


while True:
    _, img = cap.read()
    findAruco(img)
    cv2.imshow("img", img)
    if cv2.waitKey(1) == 113:
        break

