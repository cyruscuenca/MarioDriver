import numpy as np
import win32gui
import cv2
import time
import matplotlib.pyplot as plt
from PIL import ImageGrab
from PIL import Image

# Get HWID
hwnd = win32gui.FindWindow(None, r'Dolphin 5.0 | JIT64 DC | Direct3D 11 | HLE | FPS: 60 - VPS: 60 - 100%')
win32gui.SetForegroundWindow(hwnd)

# Get window dimensions
dimensions = list(win32gui.GetWindowRect(hwnd))

# Crop window dimensions
dimensions = [dimensions[0] + 10, dimensions[1] + 35, dimensions[2] - 10, dimensions[3] - 10]

# Print for debugging
print(dimensions)

def cropFrame(frame):
    height = frame.shape[0]
    polys = np.array([
        [(-500, height), (dimensions[3] + 500, height), (300, 100)]])
    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, polys, 255)
    cv2.rectangle(mask, (50, height - 20), (150, height - 100), 0, -1, lineType=8, shift=0)
    cv2.rectangle(mask, (450, height - 50), (700, height - 250), 0, -1, lineType=8, shift=0)
    cv2.rectangle(mask, (260, height - 60), (380, height - 150), 0, -1, lineType=8, shift=0)
    frame = cv2.bitwise_and(frame, mask)

    return frame

def processFrame(frame):
    # Frame processing
    gray = cv2.cvtColor(np.copy(frame), cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    frame = cv2.Canny(blur, 50, 150)

    return frame

def placeLines(frame, params):
    slope, intercept = params
    y1 = frame.shape[0]
    y2 = int(y1*(3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)

    return np.array([x1, y1, x2, y2])

def averageLines(frame, lines):
    left = []
    right = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left.append((slope, intercept))
        else:
            right.append((slope, intercept))
    leftAverage = np.average(left, axis=0)
    rightAverage = np.average(right, axis=0)
    leftLine = placeLines(frame, leftAverage)
    rightLine = placeLines(frame, rightAverage)

    return np.array([leftLine, rightLine])

def overlayFrame(frame, lines):
    overlay = np.zeros_like(frame)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            cv2.line(overlay, (x1, y1), (x2, y2), (255, 0, 0), 2)
    return overlay

while True:
    # Set frame rate
    time.sleep(1./60)
    frame = ImageGrab.grab(dimensions)
    frame.crop(dimensions)
    # Convert frame to a format OpenCV can understand
    frame = processFrame(frame)
    original = np.copy(frame)

    # Crop out data outside of region of interest
    frame = cropFrame(frame)

    # Display frame
    lines = cv2.HoughLinesP(frame, 2, np.pi/180, 100, np.array([]), minLineLength = 14, maxLineGap = 10)
    averaged = averageLines(frame, lines)
    overlay = overlayFrame(frame, averaged)
    frame = cv2.addWeighted(original, 0.8, overlay, 1, 1)
    cv2.imshow('MarioDrive', overlay)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()