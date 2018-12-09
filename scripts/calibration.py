import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt

OPTIMIZE_ALPHA = 0.25
DEPTH_VISUALIZATION_SCALE = 2048

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def trainCalibration(left=True):
    # termination criteria


    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 8, 3), np.float32)
    objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('image/both*.png')
    w, h = 0, 0
    count = 0
    for fname in images[:30]:
    	print count
    	count = count + 1
        img = cv2.imread(fname)
        height, width, _ = img.shape
        crop = img[:, :int(width / 2), :] if left else img[:, int(width / 2):, :]
        #h, w = img.shape[:2]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners

        ret, corners = cv2.findChessboardCorners(gray, (8, 6), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            #left = cv2.drawChessboardCorners(left, (8, 6), corners2, ret)
            # cv2.imshow('img',left)
            # cv2.waitKey(500)

    return objpoints, imgpoints
    # ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
    # return mtx, dist


def trainBothCalibration():
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 8, 3), np.float32)
    objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    l_imgpoints = []  # 2d points in image plane.
    r_imgpoints = []

    images = glob.glob('image/both*.png')
    w, h = 0, 0
    for fname in images[:30]:
        img = cv2.imread(fname)
        height, width, _ = img.shape
        left = img[:, :int(width / 2), :]
        right = img[:, int(width / 2):, :]
        # crop = img[:, :int(width / 2), :] if left else img[:, int(width / 2):, :]
        #h, w = img.shape[:2]
        gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)


        # Find the chess board corners

        ret1, corners1 = cv2.findChessboardCorners(gray_left, (8, 6), None)

        ret2, corners2 = cv2.findChessboardCorners(gray_right, (8, 6), None)

        # If found, add object points, image points (after refining them)
        if ret1 and ret2:
            objpoints.append(objp)

            corners1 = cv2.cornerSubPix(gray_left, corners1, (11, 11), (-1, -1), criteria)
            l_imgpoints.append(corners1)

            corners2 = cv2.cornerSubPix(gray_right, corners2, (11, 11), (-1, -1), criteria)
            r_imgpoints.append(corners2)

            # Draw and display the corners
            #left = cv2.drawChessboardCorners(left, (8, 6), corners2, ret)
            # cv2.imshow('img',left)
            # cv2.waitKey(500)

    return objpoints, l_imgpoints, r_imgpoints
    # ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
    # return mtx, dist

def undistortImg(objpoints, l_imgpoints, r_imgpoints, image):
    img = cv2.imread(image)
    left = img[:, :int(img.shape[1] / 2), :]
    right = img[:, int(img.shape[1] / 2):, :]
    height, width = left.shape[:2]
    imgsize = left.shape[::-1]
    _, l_mtx, l_dist, _, _ = cv2.calibrateCamera(
        objpoints, l_imgpoints, (width, height), None, None)

    _, r_mtx, r_dist, _, _ = cv2.calibrateCamera(
        objpoints, r_imgpoints, (width, height), None, None)

    _, _, _, _, _, rotationMatrix, translationVector, _, _ = cv2.stereoCalibrate(
        objpoints, l_imgpoints, r_imgpoints,
        l_mtx, l_dist,
        r_mtx, r_dist,
        (width, height), None, None, None, None,
        cv2.CALIB_FIX_INTRINSIC, criteria)

    leftRectification, rightRectification, leftProjection, rightProjection, \
     dispartityToDepthMap, leftROI, rightROI = cv2.stereoRectify(
        l_mtx, l_dist,
        r_mtx, r_dist,
        (width, height), rotationMatrix, translationVector,
        None, None, None, None, None,
        cv2.CALIB_ZERO_DISPARITY, OPTIMIZE_ALPHA)


    leftMapX, leftMapY = cv2.initUndistortRectifyMap(
        l_mtx, l_dist, leftRectification,
        leftProjection, (width, height), cv2.CV_32FC1)
    rightMapX, rightMapY = cv2.initUndistortRectifyMap(
        r_mtx, r_dist, rightRectification,
        rightProjection, (width, height), cv2.CV_32FC1)

    fixedLeft = cv2.remap(left, leftMapX, leftMapY, cv2.INTER_LINEAR)
    fixedRight = cv2.remap(right, rightMapX, rightMapY, cv2.INTER_LINEAR)

    # plt.imshow(fixedLeft)
    # plt.show()
    # plt.imshow(fixedRight)
    # plt.show()

    stereoMatcher = cv2.StereoBM_create(numDisparities=128, blockSize=21)
    grayLeft = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)
    grayRight = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)
    depth = stereoMatcher.compute(grayLeft, grayRight)
    # cv2.imshow('depth', depth / DEPTH_VISUALIZATION_SCALE)
    # cv2.waitKey(0)

    left_undistort = cv2.undistort(left, l_mtx, l_dist, None, l_mtx)
    right_undistort = cv2.undistort(right, r_mtx, r_dist, None, r_mtx)
    height, width = left.shape[:2]
    concat_image = np.zeros((height, width * 2, 3), dtype=np.uint8)
    for i in range(3):
        concat_image[:, :width, i] = left_undistort[:, :, i]
        concat_image[:, width:, i] = right_undistort[:, :, i]
    plt.imshow(img)
    plt.show()
    plt.imshow(concat_image)
    plt.show()
    cv2.imwrite('image/calibresultTest00.png', concat_image)

    l = cv2.cvtColor(left_undistort, cv2.COLOR_BGR2GRAY)
    r = cv2.cvtColor(right_undistort, cv2.COLOR_BGR2GRAY)
    stereo = cv2.StereoBM_create(numDisparities=128, blockSize=21)
    disparity = stereo.compute(l, r)

    cv2.imshow('depth', disparity)# / DEPTH_VISUALIZATION_SCALE)
    cv2.waitKey(0)
    return concat_image, left_undistort, right_undistort

def combine():
    objpoints, l_imgpoints, r_imgpoints = trainBothCalibration()
    WIDTH, HEIGHT = 1280, 720

    _, l_mtx, l_dist, rvec, tvec = cv2.calibrateCamera(
        objpoints, l_imgpoints, (WIDTH, HEIGHT), None, None)

    _, r_mtx, r_dist, _, _ = cv2.calibrateCamera(
        objpoints, r_imgpoints, (WIDTH, HEIGHT), None, None)

    return l_mtx, l_dist, r_mtx, r_dist


if __name__ == "__main__":
    # l_objpoints, left_imgpoints = trainCalibration()
    # r_objpoints, right_imgpoints = trainCalibration(left=False)


    # objpoints, l_imgpoints, r_imgpoints = trainBothCalibration()
    #
    # undistortImg(objpoints, l_imgpoints, r_imgpoints, "image/data12.png")
    #
    # exit()
    l_mtx, l_dist, r_mtx, r_dist = combine()
    cap = cv2.VideoCapture(1)
    while True and cap.read()[0]:
        ret, frame = cap.read()
        #cv2.imshow('frame', frame)

        left = frame[:, :int(frame.shape[1] / 2), :]
        right = frame[:, int(frame.shape[1] / 2):, :]
        left_undistort = cv2.undistort(left, l_mtx, l_dist, None, l_mtx)
        right_undistort = cv2.undistort(right, r_mtx, r_dist, None, r_mtx)
        height, width = left.shape[:2]
        concat_image = np.hstack((left_undistort,right_undistort))

        cv2.imshow('frame', concat_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
