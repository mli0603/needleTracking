import cv2
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal
import math
from scipy.ndimage import filters


def non_max_suppression(R, window=(10, 10)):
    """
    This function will extract the local maximum of a given matrix.
    Args:
        R (ndarray matrix): the matrix needed to be process.
        window (tupel): non_max_suppression window size
    Returns:
        The ndarray with same shape as R.
    """
    maxH = filters.maximum_filter(R, window)
    R = R * (R == maxH)
    return np.array(R)


def gaussian_filter2d(window_shape=(3, 3), sigma=0.5):
    m, n = [(ss - 1.0) / 2.0 for ss in window_shape]
    y, x = np.ogrid[-m:m + 1, -n:n + 1]

    h = np.exp(-(x * x + y * y) / (2. * sigma ** 2))
    h[h < np.finfo(h.dtype).eps * h.max()] = 0

    return h / h.sum() if h.sum() != 0 else h

def harris_corner(image):

    img = image.astype(np.float64)
    gauss2d = gaussian_filter2d(window_shape=(3, 3), sigma=1)

    img = signal.convolve2d(img, gauss2d, mode="same", boundary="symm")

    dy, dx = np.gradient(img)
    sobel_x = np.array([[-1, -2, 0, 2, 1],
                        [-2, -3, 0, 3, 2],
                        [-3, -5, 0, 5, 3],
                        [-2, -3, 0, 3, 2],
                        [-1, 2, 0, 2, 1]])

    sobel_y = np.array([[1, 2, 3, 2, 1],
                        [2, 3, 5, 3, 2],
                        [0, 0, 0, 0, 0],
                        [-2, -3, -5, -3, -2],
                        [-1, -2, -3, -2, -1]])

    dx = signal.convolve2d(dx, sobel_x, mode="same", boundary="symm")
    dy = signal.convolve2d(dy, sobel_y, mode="same", boundary="symm")
    ixx = dx ** 2
    ixy = dy * dx
    iyy = dy ** 2

    gauss2d = gaussian_filter2d(window_shape=(15, 15), sigma=1)
    ixx = signal.convolve2d(ixx, gauss2d, mode="same", boundary="symm")
    iyy = signal.convolve2d(iyy, gauss2d, mode="same", boundary="symm")
    ixy = signal.convolve2d(ixy, gauss2d, mode="same", boundary="symm")

    k = 0.05
    trace = ixx + iyy
    det = ixx * iyy - ixy ** 2

    mat = non_max_suppression(det - k * (trace ** 2))

    thresh = mat.max() / 800
    pixel_coords = list(map(tuple, np.argwhere(mat > thresh)))
    return pixel_coords


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # plt.imshow(img)
        # plt.show()

        feature_list = harris_corner(img)

        dis_img = img.copy()
        for i in range(len(feature_list)):
            cv2.circle(dis_img, (feature_list[i][1], feature_list[i][0]),
                       10, (255, 0, 0), 2)
        cv2.imshow('frame', dis_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cap.release()
    cv2.destroyAllWindows()
