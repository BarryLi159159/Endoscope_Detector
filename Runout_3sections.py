import cv2
import numpy as np
import csv
import json
import os
import tkinter as tk
from PIL import Image, ImageTk
import PIL
from tkinter import font as tkFont

# Constants and calibration file paths
blur_value = 5
block_size = 9
C = 2
Dila = 1
section_width = None  # Will be set based on the image width



# Function to load calibration data from a JSON file
def load_calibration_data(filename):
    with open(filename, 'r') as f:
        calibration_data = json.load(f)
    camera_matrix = np.array(calibration_data['Camera_Matrix'])
    dist_coeffs = np.array(calibration_data['Distortion_Coefficients'])
    ppmm = (calibration_data['PPMMx'], calibration_data['PPMMy'])
    return camera_matrix, dist_coeffs, ppmm


# Function to capture the sharpest frame from the video stream
def capture_sharpest_frame():
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
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
        variance = cv2.Laplacian(gray, cv2.CV_64F).var()
        frames.append(frame)
        variances.append(variance)

    cap.release()
    sharpest_frame = frames[np.argmax(variances)]
    return sharpest_frame


# Function to convert pixel values to millimeters using calibration data
def pixel_to_mm(pixel_value, ppmm):
    return pixel_value / ppmm


def detect_endoscope_nozzle(image, blur_value, deviation_threshold, calibrations):
    global section_width
    section_width = image.shape[1] // 3  # Divide the image into three sections horizontally

    # Extract the ROI from the image
    roi_image = image[roi_coords[0]:roi_coords[2], roi_coords[1]:roi_coords[3]].copy()

    # Image processing within the ROI
    gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_value, blur_value), 0)
    thresh = cv2.adaptiveThreshold(blurred, 250, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, block_size, C)
    kernel = np.ones((Dila, Dila), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=1)

    # Contour detection
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # Initialize variables to store the first intersection and significant deviation points
    first_intersection = None
    significantly_deviated_points = []

    # Draw the ROI rectangle on the image
    cv2.rectangle(image, (roi_coords[1], roi_coords[0]), (roi_coords[3], roi_coords[2]), (0, 255, 0), 2)

    # If contours are detected, process the largest one (assumed to be the endoscope)
    if contours:
        endoscope_contour = contours[0]
        for x in range(0, roi_image.shape[1], 10):  # Iterate over the width of the ROI
            for y in range(roi_image.shape[0] - 1, -1, -1):  # Iterate from bottom to top of the ROI
                if cv2.pointPolygonTest(endoscope_contour, (x, y), False) == 1:
                    global_x = x + roi_coords[1]  # Convert x to global coordinates
                    section_index = global_x // section_width  # Determine which section the point belongs to
                    _, _, (PPMMw, PPMMh) = calibrations[
                        section_index]  # Get the correct calibration data for the section

                    if first_intersection is None:
                        first_intersection = (global_x, y + roi_coords[0])

                        # Calculate the deviation in pixels and then convert to millimeters
                    deviation_pixels = first_intersection[1] - (y + roi_coords[0])
                    deviation_mm = pixel_to_mm(deviation_pixels, PPMMh)

                    if abs(deviation_mm) > deviation_threshold:
                        significantly_deviated_points.append((global_x, deviation_mm))

                    break  # Stop the loop after finding the first intersection point for this x

    # Return the first intersection and the list of points with significant deviation
    return first_intersection, significantly_deviated_points


def remove_outliers(points):
    deviations = [point[1] for point in points]
    if not deviations:
        return []

    Q1, Q3 = np.percentile(deviations, [25, 75])
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    return [point for point in points if lower_bound <= point[1] <= upper_bound]


# Load calibration data for different sections
calibration_base_path = "D:\MQP endoscope pic/"
calibration_files = [
    calibration_base_path + 'calibration_section1.json',
    calibration_base_path + 'calibration_section2.json',
    calibration_base_path + 'calibration_section3.json'
]
calibrations = [load_calibration_data(f) for f in calibration_files]

# Capture the sharpest image from the webcam
img = capture_sharpest_frame()

# Apply undistortion to the captured image using the first set of calibration data
undistorted_img = cv2.undistort(img, calibrations[0][0], calibrations[0][1])

# Define the ROI (Region of Interest) where the endoscope is expected to be located
MANUAL_ROI_START_X = 100
MANUAL_ROI_START_Y = 150
MANUAL_ROI_WIDTH = 550
MANUAL_ROI_HEIGHT = 120
roi_coords = (
MANUAL_ROI_START_Y, MANUAL_ROI_START_X, MANUAL_ROI_START_Y + MANUAL_ROI_HEIGHT, MANUAL_ROI_START_X + MANUAL_ROI_WIDTH)

# Detect the endoscope nozzle and measure deviations
first_point, significant_points_pixels = detect_endoscope_nozzle(undistorted_img, blur_value, 2, calibrations)

# Convert pixel coordinates to millimeters using calibration data
significant_points_mm = [
    (pixel_to_mm(x + roi_coords[1], calibrations[(x + roi_coords[1]) // section_width][2][0]),  # x-coord conversion
     pixel_to_mm(deviation, calibrations[(x + roi_coords[1]) // section_width][2][1]))  # y-coord conversion
    for x, deviation in significant_points_pixels
]

# Filter out the outliers and check if the endoscope is bent or straight
filtered_points_mm = remove_outliers(significant_points_mm)
is_bent = len(filtered_points_mm) > 2

# Find the point with the largest deviation
max_deviation_point_mm = max(filtered_points_mm, key=lambda point: abs(point[1]), default=None)

# Save the filtered significantly deviated points into a CSV file
csv_filename = "filtered_points_mm.csv"
with open(csv_filename, "w", newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["X (mm)", "Deviation from Origin Y (mm)"])
    writer.writerows(filtered_points_mm)

# Create the Tkinter window and display the results
root = tk.Tk()
root.title("Endoscope Status UI")

# Create and pack the UI components
status_text = "Bent" if is_bent else "Straight"
endoscope_status_label = tk.Label(root, text=f"Endoscope Status: {status_text}")
endoscope_status_label.pack(pady=10)

if max_deviation_point_mm:
    max_runout_label = tk.Label(root, text=f"Max Runout (mm): {max_deviation_point_mm[1]}")
    max_runout_label.pack(pady=10)


# Function to display image in Tkinter canvas
def display_image_on_canvas(image, canvas):
    image = Image.fromarray(image)
    # Use Image.Resampling.LANCZOS for Pillow 8.0.0 and newer, or Image.LANCZOS for older versions
    image.thumbnail((500, 500), Image.Resampling.LANCZOS)  # Resize image to fit the canvas
    photo = ImageTk.PhotoImage(image)
    canvas.create_image(0, 0, anchor=tk.NW, image=photo)
    canvas.image = photo  # Keep a reference so it's not garbage collected


# Create a canvas to display the image
canvas = tk.Canvas(root, width=500, height=500)
canvas.pack()

# Convert the BGR image to RGB and display it on the canvas
rgb_img = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2RGB)
display_image_on_canvas(rgb_img, canvas)

root.mainloop()
