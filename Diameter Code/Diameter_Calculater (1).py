import cv2
import numpy as np
import csv
import json

blur_value = 1
block_size = 11
C = 2
Dila = 1
Cannylow = 50
Cannyhigh = 150

ROI_X = 700
ROI_Y = 400
ROI_WIDTH = 300
ROI_HEIGHT = 200
def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()

def capture_sharpest_frame():
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
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

filename = r'C:\Users\Administrator\Downloads\CalibrationData5.json'
with open(filename, 'r') as f:
    calibration_data = json.load(f)


camera_matrix = np.array(calibration_data['Camera_Matrix'])
dist_coeffs = np.array(calibration_data['Distortion_Coefficients'])
PPMMw = calibration_data['PPMMx']
PPMMh = calibration_data['PPMMy']
PIXEL_TO_MM_CONVERSION = PPMMh
undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)
img = undistorted_img


def find_upper_boundary_intersection(x, contour):
    for y in range(ROI_Y, ROI_Y + ROI_HEIGHT):
        if cv2.pointPolygonTest(contour, (x, y), False) == 1:
            return (x, y)
    return None

def find_lower_boundary_intersection(x, contour):
    for y in range(ROI_Y + ROI_HEIGHT - 1, ROI_Y - 1, -1):
        if cv2.pointPolygonTest(contour, (x, y), False) == 1:
            return (x, y)
    return None

def detect_endoscope_nozzle(image, blur_value, deviation_threshold):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_value, blur_value), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block_size, C)
    kernel = np.ones((Dila, Dila), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=1)
    cv2.imshow("countor", dilated)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    vertical_spacing = 10
    upper_intersections = []
    lower_intersections = []

    if len(contours) > 0:
        endoscope_contour = contours[0]

        for x in range(ROI_X, ROI_X + ROI_WIDTH, vertical_spacing):
            upper_intersection = find_upper_boundary_intersection(x, endoscope_contour)
            lower_intersection = find_lower_boundary_intersection(x, endoscope_contour)

            if upper_intersection:
                upper_intersections.append(upper_intersection)
                cv2.line(image, (x, ROI_Y), upper_intersection, (255, 0, 0), 1)
                cv2.circle(image, upper_intersection, 3, (0, 255, 0), -1)

            if lower_intersection:
                lower_intersections.append(lower_intersection)
                cv2.line(image, (x, ROI_Y + ROI_HEIGHT), lower_intersection, (0, 255, 255), 1)
                cv2.circle(image, lower_intersection, 3, (0, 0, 255), -1)

    # Calculating the distance difference
    differences = []
    for upper, lower in zip(upper_intersections, lower_intersections):
        if upper[0] == lower[0]:
            difference = abs(upper[1] - lower[1])
            differences.append(difference)

    # Compute average
    avg_difference = sum(differences) / len(differences) if differences else 0
    print(f"Average difference between paired points: {avg_difference} pixels")
    avg_difference_mm = avg_difference / PPMMw
    print(f"Average difference between paired points: {avg_difference_mm} mm")
    # Draw ROI for visualization purposes
    cv2.rectangle(image, (ROI_X, ROI_Y), (ROI_X + ROI_WIDTH, ROI_Y + ROI_HEIGHT), (255, 255, 0), 2)
    cv2.imshow("Detected Nozzle with Upper and Lower Boundary Intersections", image)

detect_endoscope_nozzle(undistorted_img, blur_value, 2)
cv2.waitKey(0)
cv2.destroyAllWindows()