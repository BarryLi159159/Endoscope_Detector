import cv2
import numpy as np
import json
from pypylon import pylon

# Parameters
blur_value = 7
block_size = 7
C = 2
Dila = 1

# ROI_X = 1000
# ROI_Y = 600
# ROI_WIDTH = 100
# ROI_HEIGHT = 200




def variance_of_laplacian(image):
    """
    Calculate the Laplacian variance of an image.
    """
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

def pixel_to_mm(pixel_value, PPMMh):
    """
    Convert a pixel measurement to millimeters.
    """
    return pixel_value / PPMMh


def find_intersection(image, x, start_y, step, contours):
    """
    Find the first intersection of a vertical line with the contours.
    """
    y = start_y
    while 0 <= y < image.shape[0]:
        if any(cv2.pointPolygonTest(contour, (x, y), False) >= 0 for contour in contours):
            return x, y
        y += step
    return None


def detect_and_draw_contours(image, PPMMh_left, PPMMh_center, PPMMh_right,ROI_X, ROI_Y,ROI_WIDTH,ROI_HEIGHT,output):
    """
    Detect contours in the image and draw vertical lines to find intersections.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_value, blur_value), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block_size, C)

    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    mask[ROI_Y:ROI_Y + ROI_HEIGHT, ROI_X:ROI_X + ROI_WIDTH] = 255
    masked_thresh = cv2.bitwise_and(thresh, thresh, mask=mask)

    kernel = np.ones((Dila, Dila), np.uint8)
    dilated = cv2.dilate(masked_thresh, kernel, iterations=1)
    cv2.imshow("dilated", dilated)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    upper_intersections = []
    lower_intersections = []

    for x in range(ROI_X, ROI_X + ROI_WIDTH, 10):
        bottom_intersection = find_intersection(image, x, ROI_Y + ROI_HEIGHT, -1, contours)
        if bottom_intersection:
            cv2.line(image, (x, ROI_Y + ROI_HEIGHT), bottom_intersection, (0, 255, 255), 1)
            cv2.circle(image, bottom_intersection, 3, (0, 255, 255), -1)
            lower_intersections.append(bottom_intersection)

        top_intersection = find_intersection(image, x, ROI_Y, 1, contours)
        if top_intersection:
            cv2.line(image, (x, ROI_Y), top_intersection, (255, 0, 0), 1)
            cv2.circle(image, top_intersection, 3, (255, 0, 0), -1)
            upper_intersections.append(top_intersection)

    cv2.rectangle(image, (ROI_X, ROI_Y), (ROI_X + ROI_WIDTH, ROI_Y + ROI_HEIGHT), (0, 255, 0), 2)

    image_height, image_width = image.shape[:2]
    section_width = image_width // 3
    differences = []

    for upper, lower in zip(upper_intersections, lower_intersections):
        if upper[0] == lower[0]:
            pixel_difference = abs(upper[1] - lower[1])
            if upper[0] < section_width:
                mm_difference = pixel_to_mm(pixel_difference, PPMMh_left)
            elif upper[0] < 2 * section_width:
                mm_difference = pixel_to_mm(pixel_difference, PPMMh_center)
            else:
                mm_difference = pixel_to_mm(pixel_difference, PPMMh_right)
            differences.append((pixel_difference, mm_difference))

    avg_pixel_difference = sum([d[0] for d in differences]) / len(differences) if differences else 0
    avg_mm_difference = sum([d[1] for d in differences]) / len(differences) if differences else 0

    print(f"Average pixel difference: {avg_pixel_difference} pixels")
    print(f"Average mm difference: {avg_mm_difference} millimeters")

    #for i in range(1, 3):
    #    cv2.line(image, (i * section_width, 0), (i * section_width, image_height), (0, 255, 0), 1)

    #cv2.imshow("Detected Nozzle", image)
    imS = cv2.resize(image, (960, 540))                # Resize image
    cv2.imshow(output, imS)   


# Main code
EndoscopeBeginingImg = capture_sharpest_frame_basler()
EndoscopeMiddleImg = capture_sharpest_frame_basler()
EndoscopeEndImg = capture_sharpest_frame_basler()

filename = r'C:\Users\Administrator\Downloads\Basler 8mm Lens.json'
with open(filename, 'r') as f:
    calibration_data = json.load(f)

camera_matrix = np.array(calibration_data['Camera_Matrix'])
dist_coeffs = np.array(calibration_data['Distortion_Coefficients'])
PPMMw = calibration_data['PPMMx']
PPMMh = calibration_data['PPMMy']
PPMMh_left = PPMMh
PPMMh_center = PPMMh
PPMMh_right = PPMMh

EndoscopeBeginingUndistorted_img = cv2.undistort(EndoscopeBeginingImg, camera_matrix, dist_coeffs)
EndoscopeMiddleImgUndistorted_img = cv2.undistort(EndoscopeMiddleImg, camera_matrix, dist_coeffs)
EndoscopeEndUndistorted_img = cv2.undistort(EndoscopeEndImg, camera_matrix, dist_coeffs)
detect_and_draw_contours(EndoscopeBeginingUndistorted_img, PPMMh_left, PPMMh_center, PPMMh_right,500,1000,100,300,"1")
detect_and_draw_contours(EndoscopeMiddleImgUndistorted_img, PPMMh_left, PPMMh_center, PPMMh_right,1300,1000,100,300,"2")
detect_and_draw_contours(EndoscopeEndUndistorted_img , PPMMh_left, PPMMh_center, PPMMh_right,2400,1000,100,300,"3")
cv2.waitKey(0)
cv2.destroyAllWindows()
