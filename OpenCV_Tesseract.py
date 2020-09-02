import cv2
import numpy as np
import pytesseract

# Path for Mac
pytesseract.pytesseract.tesseract_cmd = r'/usr/local/Cellar/tesseract/4.1.1/bin/tesseract'

# load image
img = cv2.imread('PATH/TO/MARKER/J.png')

# lower (0-10) and upper (170-180) red values in HSV
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
m0 = cv2.inRange(img_hsv, np.array([0, 50, 50]), np.array([10, 255, 255]))
m1 = cv2.inRange(img_hsv, np.array([170, 50, 50]), np.array([180, 255, 255]))
mask_red = m0 + m1

# extract red from image and convert to greyscale
img_red = cv2.bitwise_and(img, img, mask = mask_red)
r, img_t = cv2.threshold(cv2.cvtColor(img_red, cv2.COLOR_BGR2GRAY), 50, 255,
    cv2.THRESH_BINARY)

# find corner coordinates of the red area
contour, h = cv2.findContours(img_t, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contour = sorted(contour, key = cv2.contourArea, reverse = True)[:50]
for c in contour:
    epsilon = 0.02 * cv2.arcLength(c, True)
    corners = cv2.approxPolyDP(c, epsilon, True)
    if len(corners) == 4:
        pt4 = corners
        break

pts = pt4.reshape(4, 2)
xy_old = np.zeros((4, 2), dtype = 'float32')
xy_old_sum = np.sum(pts, 1)
xy_old[0] = pts[np.argmin(xy_old_sum)]
xy_old[2] = pts[np.argmax(xy_old_sum)]
pts = np.delete(pts, np.where((pts == xy_old[:, None]).all(-1))[1], 0)
xy_old[1] = pts[np.argmin(pts[:, 1])]
xy_old[3] = pts[np.argmax(pts[:, 1])]

xy = np.float32([[0, 0], [500, 0], [500, 500], [0, 500]])
img_out = cv2.warpPerspective(img_red, cv2.getPerspectiveTransform(xy_old, xy), 
    (500, 500))

# identify character
text = pytesseract.image_to_string(img_out, config = 
    '--psm 10 -c tessedit_char_whitelist=123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ')
print(text)
