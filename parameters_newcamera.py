import cv2
import numpy as np
from pypylon import pylon

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

def adjust_parameters(*args):
    """
    Adjust the processing parameters based on trackbar values and display the combined result.
    """
    # Retrieve the trackbar values
    lower_threshold = cv2.getTrackbarPos('Lower Threshold', 'Adjustments')
    upper_threshold = cv2.getTrackbarPos('Upper Threshold', 'Adjustments')
    blur_size = cv2.getTrackbarPos('Blur Size', 'Adjustments') * 2 + 1  # Ensuring it's always odd
    blockSize = cv2.getTrackbarPos('Adaptive BlockSize', 'Adjustments')
    C = cv2.getTrackbarPos('Adaptive C', 'Adjustments')
    dilate_size = cv2.getTrackbarPos('Dilate Size', 'Adjustments')

    # Ensure blockSize is odd and >= 3
    if blockSize % 2 == 0:
        blockSize += 1
    if blockSize < 3:
        blockSize = 3

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)

    # Apply adaptive thresholding
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY_INV, blockSize, C)

    # Apply dilation
    kernel = np.ones((dilate_size, dilate_size), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=1)

    ims = cv2.resize(dilated, (960, 540))  # Resize image
    cv2.imshow("output", ims)
    # Apply Canny edge detection on the dilated image
    #edges = cv2.Canny(dilated, lower_threshold, upper_threshold)

    # Display the final result
    #cv2.imshow('Adjusted Edges', dilated)




img = capture_sharpest_frame_basler()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.namedWindow('Adjustments')


# Create trackbars to adjust parameters
cv2.createTrackbar('Lower Threshold', 'Adjustments', 50, 255, adjust_parameters)
cv2.createTrackbar('Upper Threshold', 'Adjustments', 150, 255, adjust_parameters)
cv2.createTrackbar('Blur Size', 'Adjustments', 1, 15,
                   adjust_parameters)  # Values will be multiplied by 2 and added by 1
cv2.createTrackbar('Adaptive BlockSize', 'Adjustments', 15, 50, adjust_parameters)
cv2.createTrackbar('Adaptive C', 'Adjustments', 2, 10, adjust_parameters)
cv2.createTrackbar('Dilate Size', 'Adjustments', 1, 10, adjust_parameters)

# Call the function once to display the initial processed images
adjust_parameters()

cv2.waitKey(0)
cv2.destroyAllWindows()