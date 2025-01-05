import cv2
import numpy as np
import csv
import json
from pypylon import pylon

# Configuration values
MANUAL_ROI_START_X = 380  # X coordinate where the ROI starts
MANUAL_ROI_START_Y = 900  # Y coordinate where the ROI starts
MANUAL_ROI_WIDTH = 2500    # The width of the ROI
MANUAL_ROI_HEIGHT = 300    # The height of the ROI

ROI_X = MANUAL_ROI_START_X  # X coordinate where the ROI starts
ROI_Y = MANUAL_ROI_START_Y  # Y coordinate where the ROI starts
ROI_WIDTH = MANUAL_ROI_WIDTH   # The width of the ROI
ROI_HEIGHT = MANUAL_ROI_HEIGHT     # The height of the ROI

blur_value = 7
block_size = 9
C = 2
Dila = 1

def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()

def capture_sharpest_frame_basler():
    """
    Capture and return the sharpest frame from a Basler camera.
    """
    # Setup the camera
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    num_frames = 10
    frames = []
    variances = []

    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    while camera.IsGrabbing() and len(frames) < num_frames:
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        if grabResult.GrabSucceeded():
            # Convert to OpenCV format
            image = converter.Convert(grabResult)
            frame = image.GetArray()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            variances.append(variance_of_laplacian(gray))
            frames.append(frame)

        grabResult.Release()

    camera.Close()
    return frames[np.argmax(variances)]

img = capture_sharpest_frame_basler()

filename = r'C:\Users\Administrator\Downloads\Basler 8mm Lens.json'
with open(filename, 'r') as f:
    calibration_data = json.load(f)

camera_matrix = np.array(calibration_data['Camera_Matrix'])
dist_coeffs = np.array(calibration_data['Distortion_Coefficients'])
PPMMw = calibration_data['PPMMx']
PPMMh = calibration_data['PPMMy']

PPMMw_left = PPMMw
PPMMh_left = PPMMh

PPMMw_center = PPMMw
PPMMh_center = PPMMh

PPMMw_right = PPMMw
PPMMh_right = PPMMh

undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)
orig_image = undistorted_img

def detect_endoscope_nozzle(image, blur_value, block_size, C, Dila, ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT):
    # Image preprocessing within the ROI
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_value, blur_value), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block_size, C)

    # Create and apply a mask for the ROI
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    mask[ROI_Y:ROI_Y + ROI_HEIGHT, ROI_X:ROI_X + ROI_WIDTH] = 255
    masked_thresh = cv2.bitwise_and(thresh, thresh, mask=mask)

    # Dilate the masked threshold image
    kernel = np.ones((Dila, Dila), np.uint8)
    dilated = cv2.dilate(masked_thresh, kernel, iterations=1)
    #cv2.imshow("dilated", dilated)
    # Find contours5
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    # Draw the ROI rectangle on the original image
    cv2.rectangle(image, (ROI_X, ROI_Y), (ROI_X + ROI_WIDTH, ROI_Y + ROI_HEIGHT), (0, 255, 0), 2)
    # Draw vertical lines and find intersections

    first_intersection = None
    significantly_deviated_points = []
    vertical_spacing = 10  # spacing between lines

    # Integrate the find_intersection function
    def find_intersection(x, start_y):
        y = start_y
        while 0 <= y < image.shape[0]:
            if any(cv2.pointPolygonTest(contour, (x, y), False) >= 0 for contour in contours):
                return x, y
            y += 1  # Use a step of 1 for continuous search
        return None

    for x in range(ROI_X, ROI_X + ROI_WIDTH, vertical_spacing):
        intersection = find_intersection(x, ROI_Y)
        if intersection:
            cv2.line(image, (x, ROI_Y), intersection, (0, 255, 255), 1)  # Draw line downwards until the contour
            if first_intersection is None:
                first_intersection = intersection
                cv2.circle(image, first_intersection, 5, (255, 0, 0), -1)  # Mark the first intersection point

            deviation_y = first_intersection[1] - intersection[1]
            if abs(deviation_y) > 2:
                cv2.circle(image, intersection, 3, (0, 0, 255), -1)  # Mark significant deviation points
            significantly_deviated_points.append((intersection[0], deviation_y))

    # Draw reference line
    if first_intersection:
        cv2.line(image, (ROI_X, first_intersection[1]), (ROI_X + ROI_WIDTH, first_intersection[1]), (255, 255, 0), 2)

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


# Define the auto ROI
roi_coords = (MANUAL_ROI_START_Y, MANUAL_ROI_START_X, MANUAL_ROI_START_Y + MANUAL_ROI_HEIGHT, MANUAL_ROI_START_X + MANUAL_ROI_WIDTH)
first_point, significant_points_pixels = detect_endoscope_nozzle(
    orig_image,
    blur_value,
    block_size,
    C,
    Dila,
    roi_coords[1],  # ROI_X
    roi_coords[0],  # ROI_Y
    roi_coords[3] - roi_coords[1],  # ROI_WIDTH
    roi_coords[2] - roi_coords[0]   # ROI_HEIGHT
)


def convert_to_mm(x, y, image_width):
    section_width = image_width // 3
    if x < section_width:
        return x / PPMMw_left, y / PPMMh_left
    elif x < 2 * section_width:
        return x / PPMMw_center, y / PPMMh_center
    else:
        return x / PPMMw_right, y / PPMMh_right

image_width = orig_image.shape[1]  # Get the width of the whole image
significant_points_mm = [convert_to_mm(x, deviation, image_width) for x, deviation in significant_points_pixels]

filtered_points_mm = remove_outliers(significant_points_mm)

if len(filtered_points_mm) > 2:
    print("The endoscope is bent")
elif len(filtered_points_mm) <= 2:
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
if len(filtered_points_mm) > 3:
    endoscope_status_output.config(text="The endoscope is bent")
elif len(filtered_points_mm) < 3:
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

imS = cv2.resize(orig_image, (960, 540))    # Resize image
display_image_on_canvas(imS)
root.mainloop()