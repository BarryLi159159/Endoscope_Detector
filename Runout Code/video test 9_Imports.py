import cv2
import numpy as np
import serial
import time
import csv
import json
from pypylon import pylon
import tkinter as tk
import SimplifiedControls
# Initialize serial connection
#arduino = serial.Serial('COM11', 9600, timeout=0.5)
#time.sleep(0.5)  # Ensure connection is established

# Define default ROI and processing parameters
ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT = 900, 800, 3000, 500
blur_value, block_size, C, Dila = 9, 7, 2, 1

# Load camera calibration data
calibration_file = r"C:\Users\GMShe\Pictures\Balser Calibration 2.21.24\Basler Calibration 2.21.2024.json"
with open(calibration_file, 'r') as f:
    calibration_data = json.load(f)
PPMMw = calibration_data['PPMMx']
PPMMh = calibration_data['PPMMy']
camera_matrix = np.array(calibration_data['Camera_Matrix'])
dist_coeffs = np.array(calibration_data['Distortion_Coefficients'])

def capture_and_adjust_roi():
    global ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT

    # Initialize the pylon camera
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    if camera.IsGrabbing():
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            image = converter.Convert(grabResult)
            frame = image.GetArray()
            grabResult.Release()
            camera.Close()
        else:
            print("Error: Could not capture an image.")
            camera.Close()
            return None
    else:
        print("Error: Camera is not grabbing.")
        camera.Close()
        return None

    resized_image = cv2.resize(frame, (960, 540))  # Resize image for ROI adjustment
    cv2.namedWindow("Select ROI")

    def update_roi(event, x, y, flags, param):
        global ROI_X, ROI_Y
        if event == cv2.EVENT_LBUTTONDOWN:
            # Adjust x, y coordinates according to resized image
            ROI_X, ROI_Y = int(x * (frame.shape[1] / 960)), int(y * (frame.shape[0] / 540))
            print(f"Updated ROI_X: {ROI_X}, ROI_Y: {ROI_Y}")  # Debug print

    cv2.setMouseCallback("Select ROI", update_roi)

    while True:
        roi_image = resized_image.copy()
        key = cv2.waitKey(1) & 0xFF
        if key == ord('a'):  # 'A' key
            ROI_X = max(0, ROI_X - 10)  # Move ROI left
        elif key == ord('d'):  # 'D' key
            ROI_X = min(resized_image.shape[1] - ROI_WIDTH + 4000, ROI_X + 10)  # Move ROI right
        elif key == 13:  # Enter key
            break

        # Draw ROI rectangle on the resized image
        cv2.rectangle(roi_image,
                      (int(ROI_X * (960 / frame.shape[1])), int(ROI_Y * (540 / frame.shape[0]))),
                      (int((ROI_X + ROI_WIDTH) * (960 / frame.shape[1])),
                       int((ROI_Y + ROI_HEIGHT) * (540 / frame.shape[0]))),
                      (0, 255, 0), 2)
        cv2.imshow("Select ROI", roi_image)

    cv2.destroyAllWindows()
    return ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT


# Call capture_and_adjust_roi() to adjust the ROI before starting video processing
ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT = capture_and_adjust_roi()


highest_point_time = 0  # Time when the highest point was detected
rotation_started = False
rotation_completed = False
rotation_start_time = 0  # Time when rotation started


def rotate_to_highest_point(angle_of_highest_point):
    SimplifiedControls.Actuation(4,angle_of_highest_point,"Clockwise")
    #rotate_endoscope(angle_of_highest_point, 'Forward')
    print(f"Rotated to {angle_of_highest_point:.2f} degrees to inspect highest point")


def process_frame(frame, current_time):
    global highest_point_within_roi, highest_point_time, rotation_completed

    # Convert frame to grayscale and apply Gaussian blur
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_value, blur_value), 0)
    # Apply adaptive threshold and mask for ROI
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block_size, C)
    mask = np.zeros_like(gray)
    mask[ROI_Y:ROI_Y + ROI_HEIGHT, ROI_X:ROI_X + ROI_WIDTH] = 255
    masked_thresh = cv2.bitwise_and(thresh, thresh, mask=mask)
    # Dilate the image to connect components
    dilated = cv2.dilate(masked_thresh, np.ones((Dila, Dila), np.uint8), iterations=1)
    # Find contours
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Assuming the largest contour is the endoscope
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        # Find the highest point on the largest contour
        highest_point = min(largest_contour, key=lambda x: x[0][1])
        highest_point = (highest_point[0][0], highest_point[0][1])  # Convert to (x, y) format

        if highest_point[1] < highest_point_within_roi[1]:
            highest_point_within_roi = highest_point
            highest_point_time = current_time

            # Draw the highest point on the frame
        if highest_point_within_roi[1] != np.inf:
            cv2.circle(frame, highest_point_within_roi, 15, (0, 0, 255), -1)

            # Draw ROI rectangle
    cv2.rectangle(frame, (ROI_X, ROI_Y), (ROI_X + ROI_WIDTH, ROI_Y + ROI_HEIGHT), (0, 255, 0), 2)

    # Resize and display the frame
    resized_frame = cv2.resize(frame, (960, 540))
    cv2.imshow('Original Video with ROI', resized_frame)


    # Check if rotation is completed
    if SimplifiedControls.arduino.inWaiting() > 0 and SimplifiedControls.arduino.readline().decode().strip() == "Done" and not rotation_completed:
        rotation_completed = True
        print(f"Rotation completed at: {time.time()}")


def start_video_processing_with_rotation():
    global highest_point_within_roi, highest_point_time, rotation_start_time, rotation_completed
    highest_point_within_roi = (0, np.inf)
    rotation_started = False
    rotation_completed = False

    # Initialize the camera and start grabbing
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    # Print the current time as the video capture start time
    video_capture_start_time = time.time()
    print(f"Video capture start time: {video_capture_start_time}")

    # Start grabbing frames from the camera
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    # Introduce a slight delay to ensure the camera is ready (if necessary)
    time.sleep(0.1)  # Adjust based on your system's needs

    # Send the rotation command and record the rotation start time
    SimplifiedControls.Actuation(4,360,"Clockwise")
    #rotate_endoscope(360, 'Forward')
    rotation_start_time = time.time()
    print(
        f"Rotation command sent at: {rotation_start_time}, delay: {rotation_start_time - video_capture_start_time:.2f} seconds")

    while camera.IsGrabbing() and not rotation_completed:
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            image = converter.Convert(grabResult)
            frame = image.GetArray()
            current_time = time.time() - rotation_start_time
            process_frame(frame, current_time)
            grabResult.Release()

            if cv2.waitKey(1) & 0xFF == ord('q') or rotation_completed:
                break
        else:
            print("Failed to grab image.")

    calculate_and_rotate_to_highest_point()

    # Cleanup
    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows()

def calculate_and_rotate_to_highest_point():
    global highest_point_within_roi, highest_point_time, rotation_start_time
    rotation_duration = time.time() - rotation_start_time
    if highest_point_time != 0:
        angle_of_highest_point = (highest_point_time / rotation_duration) * 360
    else:
        angle_of_highest_point = 0
    print(f"Highest point: {highest_point_within_roi}, Angle: {angle_of_highest_point:.2f} degrees")
    rotate_to_highest_point(angle_of_highest_point)

if __name__ == "__main__":
    start_video_processing_with_rotation()


def capture_and_undistort_image(camera_matrix, dist_coeffs, show_roi=True):
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()

    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    if camera.IsGrabbing():
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            image = converter.Convert(grabResult)
            frame = image.GetArray()
            grabResult.Release()
            camera.Close()
            # Undistort the captured frame
            undistorted_img = cv2.undistort(frame, camera_matrix, dist_coeffs)
            #if show_roi:
                # Draw ROI rectangle on the undistorted image
                #cv2.rectangle(undistorted_img, (ROI_X, ROI_Y), (ROI_X + ROI_WIDTH, ROI_Y + ROI_HEIGHT), (0, 255, 0), 2)
            return undistorted_img
        else:
            camera.Close()
            raise IOError("Failed to grab frame")
    camera.Close()
    raise IOError("Camera is not grabbing")


def detect_endoscope_nozzle(undistorted_img, blur_value, block_size, C, Dila, ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT):


    # Image preprocessing within the ROI
    gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_value, blur_value), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block_size, C)

    # Create and apply a mask for the ROI
    mask = np.zeros(undistorted_img.shape[:2], dtype=np.uint8)
    mask[ROI_Y:ROI_Y + ROI_HEIGHT, ROI_X:ROI_X + ROI_WIDTH] = 255
    masked_thresh = cv2.bitwise_and(thresh, thresh, mask=mask)

    # Dilate the masked threshold image
    kernel = np.ones((Dila, Dila), np.uint8)
    dilated = cv2.dilate(masked_thresh, kernel, iterations=1)

    #cv2.imshow("dilated", dilated)
    # Find contours
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    # Draw the ROI rectangle on the original image
    cv2.rectangle(undistorted_img, (ROI_X, ROI_Y), (ROI_X + ROI_WIDTH, ROI_Y + ROI_HEIGHT), (0, 255, 0), 2)
    # Draw vertical lines and find intersections

    first_intersection = None
    significantly_deviated_points = []
    vertical_spacing = 20  # spacing between lines

    # Integrate the find_intersection function
    def find_intersection(x, start_y):
        y = start_y
        while 0 <= y < undistorted_img.shape[0]:
            if any(cv2.pointPolygonTest(contour, (x, y), False) >= 0 for contour in contours):
                return x, y
            y += 1
        return None

    for x in range(ROI_X, ROI_X + ROI_WIDTH, vertical_spacing):
        intersection = find_intersection(x, ROI_Y)
        if intersection:
            cv2.line(undistorted_img, (x, ROI_Y), intersection, (0, 255, 255), 1)  # Draw line downwards until the contour
            if first_intersection is None:
                first_intersection = intersection
                cv2.circle(undistorted_img, first_intersection, 5, (255, 0, 0), -1)  # Mark the first intersection point

            deviation_y = first_intersection[1] - intersection[1]
            if abs(deviation_y) > 2:
                cv2.circle(undistorted_img, intersection, 3, (0, 0, 255), -1)  # Mark significant deviation points
            significantly_deviated_points.append((intersection[0], deviation_y))

    # Draw reference line
    if first_intersection:
        cv2.line(undistorted_img, (ROI_X, first_intersection[1]), (ROI_X + ROI_WIDTH, first_intersection[1]), (255, 255, 0), 2)

    imS = cv2.resize(undistorted_img, (960, 540))  # Resize image
    cv2.imshow("Detected Nozzle with Significant Deviation Points", imS)

    return first_intersection, significantly_deviated_points


def remove_outliers(points):
    # Extract deviations (y-values)
    deviations = [point[1] for point in points]

    # Return an empty list if there are no points
    if not deviations:
        return []

        # Calculate the first and third quartiles (Q1, Q3)
    Q1 = np.percentile(deviations, 25)
    Q3 = np.percentile(deviations, 75)
    IQR = Q3 - Q1

    # Define lower and upper boundary for filtering
    lower_boundary = Q1 - 1.5 * IQR
    upper_boundary = Q3 + 1.5 * IQR

    # Filter out points outside the boundaries
    filtered_points = [point for point in points if lower_boundary <= point[1] <= upper_boundary]
    return filtered_points


def convert_to_mm(x, y):
    return x / PPMMw, y / PPMMh


undistorted_img = capture_and_undistort_image(camera_matrix, dist_coeffs)


def FindRunout():

    orig_image = undistorted_img
    # Define the ROI
    roi_coords = (ROI_Y, ROI_X, ROI_Y + ROI_HEIGHT, ROI_X + ROI_WIDTH)
    first_point, significant_points_pixels = detect_endoscope_nozzle(
        orig_image,
        blur_value,
        block_size,
        C,
        Dila,
        roi_coords[1],  # ROI_X
        roi_coords[0],  # ROI_Y
        roi_coords[3] - roi_coords[1],  # ROI_WIDTH
        roi_coords[2] - roi_coords[0]  # ROI_HEIGHT
    )

    # Convert points from pixels to mm
    significant_points_mm = [convert_to_mm(x, y) for x, y in significant_points_pixels]

    # Remove outliers
    filtered_points_mm = remove_outliers(significant_points_mm)

    # Analyze the filtered points
    if len(filtered_points_mm) > 2:
        endoscope_status = "The endoscope is bent"
    else:
        endoscope_status = "The endoscope is straight"

    max_deviation_point_mm = max(filtered_points_mm, key=lambda point: abs(point[1]), default=None)
    if max_deviation_point_mm:
        index_of_max_deviation = significant_points_mm.index(max_deviation_point_mm)
        max_deviation_point_pixel = significant_points_pixels[index_of_max_deviation]

    print("Largest absolute Y deviation (mm):", max_deviation_point_mm[1])
    print("Largest absolute Y deviation (pixels):", max_deviation_point_pixel[1])

    # Create a simple Tkinter window to display the results
    root = tk.Tk()
    root.title("Runout Analysis Results")

    status_label = tk.Label(root, text=endoscope_status)
    status_label.pack()

    mm_label = tk.Label(root, text=f"Largest absolute Y deviation (mm): {max_deviation_point_mm[1]:.2f}")
    mm_label.pack()

    pixel_label = tk.Label(root, text=f"Largest absolute Y deviation (pixels): {max_deviation_point_pixel[1]:.2f}")
    pixel_label.pack()

    # Start the Tkinter event loop
    root.mainloop()

    # Save the filtered significantly deviated points into a CSV file
    with open("filtered_points_mm.csv", "w", newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["X (mm)", "Deviation from Origin Y (mm)"])
        writer.writerows(filtered_points_mm)

    return filtered_points_mm, max_deviation_point_mm, orig_image


FindRunout()
























