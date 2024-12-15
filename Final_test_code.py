import pygame
from picamera2 import Picamera2, Preview
import time
import cv2
import numpy as np
import RPi.GPIO as gpio

# Stepper motor setup for the right motor
direction_pin_right = 20
pulse_pin_right = 21
cw_direction = 0 
ccw_direction = 1

# Stepper motor setup for the left motor
direction_pin_left = 13  
pulse_pin_left = 12

gpio.setmode(gpio.BCM)
gpio.setup(direction_pin_right, gpio.OUT)
gpio.setup(pulse_pin_right, gpio.OUT)
gpio.setup(direction_pin_left, gpio.OUT)
gpio.setup(pulse_pin_left, gpio.OUT)

# Initialize pygame mixer
pygame.mixer.init()
# alert_sound = pygame.mixer.Sound('alert.wav')

# PiCamera2 setup
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280, 720)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 25
picam2.preview_configuration.align()
picam2.start()

width = 1280
height = 720
region_of_interest_vertices = [
    (0.12 * width, height),
    (0.05 * width, 0.1 * height),
    (0.96 * width, 0.1 * height),
    (0.9 * width, height)
]

# Region of Interest
def ROI(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_img = cv2.bitwise_and(img, mask)
    return masked_img

# Function for annotating the frame with the detection
def line_detected(frame, side):
    if side == "both":
        text1 = "Left Detected"
        text2 = "Right Detected"
        cv2.putText(frame, text1, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
        cv2.putText(frame, text2, (900, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
    elif side == "left":
        text1 = "Left not detected"
        text2 = "Right detected"
        cv2.putText(frame, text1, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        cv2.putText(frame, text2, (900, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
    elif side == "right":
        text1 = "Right not detected"
        text2 = "Left detected"
        cv2.putText(frame, text1, (900, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        cv2.putText(frame, text2, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
    else:
        text = "No lines detected"
        cv2.putText(frame, text, (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

# Function to rotate the right motor
def rotate_motor_right(clockwise=True):
    gpio.output(direction_pin_right, cw_direction if clockwise else ccw_direction)
    gpio.output(pulse_pin_right, gpio.HIGH)
    time.sleep(0.000001)
    gpio.output(pulse_pin_right, gpio.LOW)
    time.sleep(0.000001)

# Function to rotate the left motor
def rotate_motor_left(clockwise=True):
    gpio.output(direction_pin_left, cw_direction if clockwise else ccw_direction)
    gpio.output(pulse_pin_left, gpio.HIGH)
    time.sleep(0.000000001)
    gpio.output(pulse_pin_left, gpio.LOW)
    time.sleep(0.000000001)

# Define the codec and create a VideoWriter object
#The below two lines of code will record the stream and store it. Comment it if no recording is needed. 
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output11.avi', fourcc, 5, (1280, 720))

try:
    while True:
        tStart = time.time()  # Start time for FPS calculation
        frame = picam2.capture_array()
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        crop_image = ROI(gray_image, np.array([region_of_interest_vertices], np.int32))
        blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)
        canny_image = cv2.Canny(blurred, 170, 240)
        crop_image1 = ROI(canny_image, np.array([region_of_interest_vertices], np.int32))

        # Probabilistic Hough Transform
        lines = cv2.HoughLinesP(crop_image1, rho=1, theta=np.pi / 180, threshold=100, minLineLength=70, maxLineGap=10)

        # Calculate the vertical axis (center) of the belt
        vertical_axis = int(0.2 * width + (0.75 * width - 0.2 * width) / 2)
        result_image = frame.copy()

        # Initialize variables to store distances to the edges
        left_detected = False
        right_detected = False

        # Determining the side of line detection
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Calculate intersection point with vertical axis
                intersection_x = (x1 + x2) // 2
                # Update variables based on the side of the line
                if intersection_x < vertical_axis:
                    left_detected = True
                else:
                    right_detected = True

        # Control the stepper motor based on line detection
        if left_detected and right_detected:
            line_detected(result_image, "both")
        elif left_detected:
            line_detected(result_image, "right")
            rotate_motor_right(clockwise=True)
        elif right_detected:
            line_detected(result_image, "left")
            rotate_motor_left(clockwise=True)
        else:
            line_detected(result_image, "nolines") # This case is not possible in real scenarios because the belt deviates to one side at a time and only one edge becomes invisible
                
        # Draw detected lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(result_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Calculate FPS
        fps = 1 / (time.time() - tStart)  # Calculate the time taken for the frame
        cv2.putText(result_image, f'FPS: {fps:.2f}', (500, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        out.write(result_image)

        # Display the result image
        small_frame = cv2.resize(result_image, (640, 360))
        cv2.imshow('Frame', small_frame)
        
        #cv2.imshow('Frame', result_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    gpio.cleanup()
    picam2.close()
    out.release()
    cv2.destroyAllWindows()
