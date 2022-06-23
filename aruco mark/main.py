import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import math


# rv2rpy & rpy2rv funcs are for matching rotation vector protocol of UR-Manipulator
def rv2rpy(rx, ry, rz):
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    kx = rx / theta
    ky = ry / theta
    kz = rz / theta
    cth = math.cos(theta)
    sth = math.sin(theta)
    vth = 1 - math.cos(theta)

    r11 = kx * kx * vth + cth
    r12 = kx * ky * vth - kz * sth
    r13 = kx * kz * vth + ky * sth
    r21 = kx * ky * vth + kz * sth
    r22 = ky * ky * vth + cth
    r23 = ky * kz * vth - kx * sth
    r31 = kx * kz * vth - ky * sth
    r32 = ky * kz * vth + kx * sth
    r33 = kz * kz * vth + cth

    beta = math.atan2(-r31, math.sqrt(r11 * r11 + r21 * r21))

    if beta > math.radians(89.99):
        beta = math.radians(89.99)
        alpha = 0
        gamma = math.atan2(r12, r22)
    elif beta < -math.radians(89.99):
        beta = -math.radians(89.99)
        alpha = 0
        gamma = -math.atan2(r12, r22)
    else:
        cb = math.cos(beta)
        alpha = math.atan2(r21 / cb, r11 / cb)
        gamma = math.atan2(r32 / cb, r33 / cb)

    return np.array([gamma, beta, alpha])


def rpy2rv(roll, pitch, yaw):
    alpha = yaw
    beta = pitch
    gamma = roll

    ca = math.cos(alpha)
    cb = math.cos(beta)
    cg = math.cos(gamma)
    sa = math.sin(alpha)
    sb = math.sin(beta)
    sg = math.sin(gamma)

    r11 = ca * cb
    r12 = ca * sb * sg - sa * cg
    r13 = ca * sb * cg + sa * sg
    r21 = sa * cb
    r22 = sa * sb * sg + ca * cg
    r23 = sa * sb * cg - ca * sg
    r31 = -sb
    r32 = cb * sg
    r33 = cb * cg

    theta = math.acos((r11 + r22 + r33 - 1) / 2)
    sth = math.sin(theta)
    kx = (r32 - r23) / (2 * sth)
    ky = (r13 - r31) / (2 * sth)
    kz = (r21 - r12) / (2 * sth)

    return np.array([(theta * kx), (theta * ky), (theta * kz)])


def rot_mat(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


def rot2eul(R):

    beta = -np.arcsin(R[2, 0])
    alpha = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
    gamma = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))

    return np.array((alpha, beta, gamma))


def move_mat(R, T):
    RT = np.zeros((3, 4))  # combined Rotation/Translation matrix
    RT[:3, :3] = R
    RT[:3, 3] = T

    return RT


def findAruco(img, ind, marker_size=4, total_markers=50, draw=True):

    if ind == 4:
        ind = 5

    detect = 0
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{marker_size}X{marker_size}_{total_markers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)

    where = np.where(ids == [ind])

    print(ids)
    print("where:", where)
    print(np.asarray(where))
    print(np.array(where))
    print(len(where))
    print(np.array(where).shape)
    print(np.array(where).T)

    a = np.array(where).T

    print(a.shape)
    # print(len(a[0]))
    if a.shape != (0, 1):
        if a.shape != (0, 2):
            if len(a[0]) == 2:
                print("Length = ", len(a[0]))
                # if np.array(where)[0][0] == 0 or np.array(where)[0][0] == 1 or np.array(where)[0][0] == 2 or np.array(
                # where)[0][0] == 3 or np.array(where)[0][0] == 4:
                print('a[0] =', a[0])
                if a[0][0] == 0 or a[0][0] == 1 or a[0][0] == 2 or a[0][0] == 3 or a[0][0] == 4:
                    print('detect!')
                    print('Object Detection Completed!')
                    detect = 1
                    print('where = ', where[0])
                    w = where[0][0]
                    print('w = ', w)

                else:
                    detect = 0
                    print('detection restart')
                    rvec = np.array([[[0, 0, 0]]])
                    tvec = np.array([[[0, 0, 0]]])

            elif len(a[0]) == 1:
                print("Len = ", len(a))
                # code to show 'No Ids' when no markers are found
                detect = 0
                print('detection restart')
                rvec = np.array([[[0, 0, 0]]])
                tvec = np.array([[[0, 0, 0]]])
        else:
            print("a is empty!")
            # code to show 'No Ids' when no markers are found
            detect = 0
            print('detection restart')
            rvec = np.array([[[0, 0, 0]]])
            tvec = np.array([[[0, 0, 0]]])
    else:
        print("a is empty!")
        # code to show 'No Ids' when no markers are found
        detect = 0
        print('detection restart')
        rvec = np.array([[[0, 0, 0]]])
        tvec = np.array([[[0, 0, 0]]])
    # font for displaying text (below)
    print('detection =', detect)
    # check if the ids list is not empty
    # if no check is added the code will crash
    if detect == 1:

        print('detection start')
        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(np.array(bboxs[w]), 0.35, mtx, dist)
        # (rvec-tvec).any() # get rid of that nasty numpy value array error

        aruco.drawAxis(img, mtx, dist, rvec, tvec, 0.1)

        # draw a square around the markers
        aruco.drawDetectedMarkers(img, bboxs)

    else:
        # code to show 'No Ids' when no markers are found
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, "Detection Failed", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        rvec = np.array([[[0, 0, 0]]])
        tvec = np.array([[[0, 0, 0]]])

    return detect, rvec, tvec


rot_tot = np.zeros(shape=(5, 3))

# Pose 0
rot_0 = np.array([-0.3584, 2.0230, 2.0209])
rot_0_m = rv2rpy(rot_0[0], rot_0[1], rot_0[2])

# Pose 1
rot_1 = np.array([-0.2111, 2.0298, 1.8462])
rot_1_m = rv2rpy(rot_1[0], rot_1[1], rot_1[2])

# Pose 2
rot_2 = np.array([-0.3797, 2.0133, 2.010])
rot_2_m = rv2rpy(rot_2[0], rot_2[1], rot_2[2])

# Pose 3
rot_3 = np.array([-0.4473, 2.026, 2.0307])
rot_3_m = rv2rpy(rot_3[0], rot_3[1], rot_3[2])

# Pose 3
rot_4 = np.array([-0.3427, 2.0172, 2.0094])
rot_4_m = rv2rpy(rot_4[0], rot_4[1], rot_4[2])

# Total Pose
rot_tot[0] = rot_0_m
rot_tot[1] = rot_1_m
rot_tot[2] = rot_2_m
rot_tot[3] = rot_3_m
rot_tot[4] = rot_4_m

# End2Cam
rot_ec = np.array([[1, 0, 0],
                   [0, 1, 0],
                   [0, 0, 1]])

# Socket Server Setting
host = "192.168.112.18"
port = 8089

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((host, port))
sock.listen(1)
print('Socket Connection Completed!')

# Camera Open
cap = cv2.VideoCapture(cv2.CAP_DSHOW + 1)
cap.isOpened()
print('Cam Open!')

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

while True:

    print('Waiting for Object Num')
    connectionSocket, addr = sock.accept()
    data = connectionSocket.recv(1024)

    print('Data Received!')
    if data:

        print('Start Pose Estimation')
        obj_n = int(data.decode('utf-8'))
        print("Data = ", obj_n)

        det = 0

        while det == 0:
            # If data is received, start estimation pose
            _, img = cap.read()
            det, rot, trans = findAruco(img, obj_n)
            cv2.imshow("img", img)
            if det == 1:
                break

# ==================================Transformation Matrix Calculation===================================================

        trans_co = np.array([trans[0][0][0], trans[0][0][1], trans[0][0][2]])
        rot_co = rot_mat(np.array([rot[0][0][0], rot[0][0][1], rot[0][0][2]]))

        trans_ec = np.array([0, 0, 1.05])

        trans_co = trans_co - trans_ec

        rot_se = rot_mat(rot_tot[obj_n])
        rot_sc = np.dot(np.array([[math.cos(math.pi/8), 0, math.sin(math.pi/8)],
                    [0, 1, 0],
                    [-math.sin(math.pi/8), 0, math.cos(math.pi/8)]
                    ]), (np.dot(rot_ec, rot_se)))

        # Relative position transformation
        trans_so = np.dot(rot_sc, trans_co)

        # Orientation of object about base {s} frame
        rot_so = np.dot(rot_sc, rot_co)

        # Rot_Mat to Euler Transformation
        rot_m_so = rot2eul(rot_so)
        rot_m_so_convert = rpy2rv(rot_m_so[0], rot_m_so[1], rot_m_so[2])

# ============================================Data Form Translation=====================================================
        tot_t = 0

        for i in range(0, 3):
            val = trans_so[2 - i]
            if val >= 0:
                sgn = 2
                val = round(val * 100)
            else:
                sgn = 1
                val = round((-val) * 100)
            tot_t = tot_t + (sgn * 10000 + val) * (100000 ** i)

        tot_r = 0

        for i in range(0, 3):
            # val = rot_m_so_convert[2 - i]
            val = 0
            if val >= 0:
                sgn = 2
                val = round(val * 1000)
            else:
                sgn = 1
                val = round((-val) * 1000)
            tot_r = tot_r + (sgn * 10000 + val) * (100000 ** i)

        tot = tot_t * (10 ** 15) + tot_r
        data_tot = str(tot)

        connectionSocket.send(data_tot.encode('utf-8'))
        print(data_tot)

        print(rot)
        print(trans)
        print('Data Send!', 'Rot_Vector =', rot_m_so_convert, 'Trans_Vector =', trans_so)
