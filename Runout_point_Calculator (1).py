import cv2
import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d
import json

# Configuration values
MANUAL_ROI_START_X = 450  # X coordinate where the ROI starts
MANUAL_ROI_START_Y = 450  # Y coordinate where the ROI starts
MANUAL_ROI_WIDTH = 1200    # The width of the ROI
MANUAL_ROI_HEIGHT = 200    # The height of the ROI


blur_value = 5
block_size = 9
C = 2
Dila = 1
Cannylow = 50
Cannyhigh = 150
def detect_endoscope_nozzle(image, roi_coords, blur_value, deviation_threshold):
    roi_image = image[roi_coords[0]:roi_coords[2], roi_coords[1]:roi_coords[3]].copy()

    # Process ROI Image
    gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_value, blur_value), 0)
    thresh = cv2.adaptiveThreshold(blurred, 250, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, block_size, C)
    kernel = np.ones((Dila, Dila), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=1)
    edge = cv2.Canny(dilated, 50, 150)
    # Contour Detection
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    cv2.imshow('Edges1', dilated)  # for contour as it is now

    # Extract significant points and draw the detected nozzle
    first_intersection = None
    significantly_deviated_points = []

    cv2.rectangle(image, (roi_coords[1], roi_coords[0]), (roi_coords[3], roi_coords[2]), (0, 255, 0),
                  2)  # ROI rectangle

    if len(contours) > 0:
        endoscope_contour = contours[0]
        for x in range(0, roi_image.shape[1], 10):
            for y in range(roi_image.shape[0] - 1, -1, -1):
                if cv2.pointPolygonTest(endoscope_contour, (x, y), False) == 1:
                    cv2.line(image, (x + roi_coords[1], y + roi_coords[0]), (x + roi_coords[1], roi_coords[2]),
                             (0, 255, 255), 1)

                    if first_intersection is None:
                        first_intersection = (x, y)
                        cv2.circle(image, (x + roi_coords[1], y + roi_coords[0]), 7, (255, 0, 0), -1)  # Origin point

                    deviation = first_intersection[1] - y
                    if abs(deviation) > deviation_threshold:
                        cv2.circle(image, (x + roi_coords[1], y + roi_coords[0]), 5, (0, 0, 255), -1)
                        significantly_deviated_points.append((x, deviation))
                    break

        cv2.line(image, (first_intersection[0] + roi_coords[1], first_intersection[1] + roi_coords[0]),
                 (roi_image.shape[1] + roi_coords[1], first_intersection[1] + roi_coords[0]), (255, 255, 0),
                 2)  # Reference Line

    cv2.imshow("Detected Nozzle with Significant Deviation Points", image)
    return first_intersection, significantly_deviated_points


def remove_outliers(points):
    # Extract deviations (y-values)
    deviations = [point[1] for point in points]

    # Check for an empty list
    if not deviations:
        return []

    Q1 = np.percentile(deviations, 25)
    Q3 = np.percentile(deviations, 75)
    IQR = Q3 - Q1

    lower_boundary = Q1 - 1.5 * IQR
    upper_boundary = Q3 + 1.5 * IQR

    # Filter out outliers
    filtered_points = [point for point in points if lower_boundary <= point[1] <= upper_boundary]
    return filtered_points


# image_path = "D:\projects\MQP\straightshaft_auto.jpg"
# orig_image = cv2.imread(image_path)
def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()


def capture_sharpest_frame():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
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


# Capture the image from the webcam
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

orig_image = undistorted_img

# Define the auto ROI
roi_coords = (MANUAL_ROI_START_Y, MANUAL_ROI_START_X, MANUAL_ROI_START_Y + MANUAL_ROI_HEIGHT, MANUAL_ROI_START_X + MANUAL_ROI_WIDTH)
first_point, significant_points_pixels = detect_endoscope_nozzle(orig_image, roi_coords, blur_value, 2)

#print("Origin (First Intersection Point) in pixels:", first_point)
#print("Origin (First Intersection Point) in mm:",(first_point[0] * PPMMw, first_point[1] * PIXEL_TO_MM_CONVERSION))

# Convert pixel coordinates to mm using the first_point as origin
significant_points_mm = [(x / PPMMw, deviation / PIXEL_TO_MM_CONVERSION) for x, deviation in significant_points_pixels]

filtered_points_mm = remove_outliers(significant_points_mm)

if len(filtered_points_mm) > 2:
    print("The endoscope is bend")
elif len(filtered_points_mm) < 2:
    print("The endoscope is straight")

max_deviation_point_mm = max(filtered_points_mm, key=lambda point: abs(point[1]), default=None)
if max_deviation_point_mm:
    index_of_max_deviation = significant_points_mm.index(max_deviation_point_mm)
    max_deviation_point_pixel = significant_points_pixels[index_of_max_deviation]

    print("Largest absolute Y deviation (mm):", max_deviation_point_mm[1])
    print("Largest absolute Y deviation (pixels):", max_deviation_point_pixel[1])

# Save the filtered significantly deviated points into a CSV file
with open("filtered_points_mm.csv", "w", newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["X (mm)", "Deviation from Origin Y (mm)"])
    writer.writerows(filtered_points_mm)



#-----------------------------------------------------------------------------------------------------------------------
import tkinter as tk
from PIL import Image, ImageTk
from tkinter import font as tkFont

# Create the main window
root = tk.Tk()
root.title("Endoscope Status UI")

customFont = tkFont.Font(family="Helvetica", size=26, weight="bold")

# Create and pack the UI components
endoscope_status_label = tk.Label(root, text="Endoscope Status:")
endoscope_status_output = tk.Label(root, text="")
endoscope_status_label.pack(pady=10)
endoscope_status_output.pack(pady=10)

max_runout_label = tk.Label(root, text="Max Runout of the Endoscope:")
max_runout_output = tk.Label(root, text="")
max_runout_label.pack(pady=10)
max_runout_output.pack(pady=10)

visualization_label = tk.Label(root, text="Visualization:")
visualization_label.pack(pady=10)
canvas = tk.Canvas(root, width=MANUAL_ROI_WIDTH, height=MANUAL_ROI_HEIGHT)
canvas.pack(pady=10)

# Instead of print statements, update the tkinter labels:
if len(filtered_points_mm) > 2:
    endoscope_status_output.config(text="The endoscope is bend")
elif len(filtered_points_mm) < 2:
    endoscope_status_output.config(text="The endoscope is straight")

if max_deviation_point_mm:
    deviation_text = f"Deviation (mm): {max_deviation_point_mm[1]}\nDeviation (pixels): {max_deviation_point_pixel[1]}"
    max_runout_output.config(text=deviation_text)


def display_image_on_canvas(image):
    height, width, _ = image.shape
    canvas.config(width=width, height=height)  # Set canvas size to image dimensions

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(image)
    image = ImageTk.PhotoImage(image)
    canvas.image = image
    canvas.create_image(0, 0, anchor=tk.NW, image=image)


display_image_on_canvas(orig_image)

root.mainloop()