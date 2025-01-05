import cv2
import numpy as np
import csv
import json

blur_value = 9
block_size = 9
C = 2
Dila = 1
Cannylow = 50
Cannyhigh = 150
#ROI_X = 250
ROI_Y = 100
ROI_WIDTH = 20
ROI_HEIGHT = 200
def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()

def capture_sharpest_frame():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    num_frames = 10
    frames = []
    variances = []

    for _ in range(num_frames):
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        variance = variance_of_laplacian(gray)
        frames.append(frame)
        variances.append(variance)
    cap.release()
    sharpest_frame = frames[np.argmax(variances)]
    return sharpest_frame

img = capture_sharpest_frame()

filename = "D:\projects\MQP\calibration_results.json"
with open(filename, 'r') as f:
    calibration_data = json.load(f)

camera_matrix = np.array(calibration_data['Camera_Matrix'])
dist_coeffs = np.array(calibration_data['Distortion_Coefficients'])
PPMMw = calibration_data['PPMMx']
PPMMh = calibration_data['PPMMy']


def pixel_to_mm(pixel_value, PPMMh):
    return pixel_value / PPMMh
undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)


def find_angles_in_contour(contour, angle_threshold, points_gap=3):
    angles = []
    for i in range(points_gap, len(contour) - points_gap):
        pt1 = contour[i - points_gap][0]
        pt2 = contour[i][0]
        pt3 = contour[i + points_gap][0]

        angle = np.abs(np.arctan2(pt3[1] - pt2[1], pt3[0] - pt2[0]) - np.arctan2(pt1[1] - pt2[1], pt1[0] - pt2[0]))
        angle = np.abs(angle * 180.0 / np.pi)

        if angle > angle_threshold:
            angles.append(pt2)
    return angles

def AutoROI(image, blur_value, angle_threshold=30, offset=5):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_value, blur_value), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block_size, C)
    kernel = np.ones((Dila, Dila), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=1)

    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    rightmost_angle_x = 0
    for contour in contours:
        angles = find_angles_in_contour(contour, angle_threshold)
        for angle in angles:
            if ROI_Y <= angle[1] <= ROI_Y + ROI_HEIGHT:
                rightmost_angle_x = max(rightmost_angle_x, angle[0])

    ROI_X = rightmost_angle_x + offset if rightmost_angle_x != 0 else 250  # Default ROI_X if no angle is found
    return ROI_X
    # Draw the ROI on the image for visualization
    cv2.rectangle(image, (ROI_X, ROI_Y), (ROI_X + ROI_WIDTH, ROI_Y + ROI_HEIGHT), (255, 0, 0), 2)
    cv2.imshow("Image with ROI", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



# Call AutoROI function
ROI_X = AutoROI(undistorted_img, blur_value)
