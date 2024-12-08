#!/usr/bin/python3
"""
Robot Vision and Communication Script
-------------------------------------
This script captures frames from a PiCamera, detects contours in the image,
determines the direction based on the largest contour, and communicates
with an Arduino via serial connection.
"""

import cv2
import numpy as np
import serial
from picamera2 import Picamera2

# Uncomment if using ultrasonic sensor and GPIO
# from gpiozero import DistanceSensor
# import RPi.GPIO as GPIO

# --- Configuration ---

# Serial port configuration
PORT = '/dev/ttyUSB0'  # Replace with the correct port for your Arduino
BAUD_RATE = 115200

# Initialize serial communication
ser = serial.Serial(PORT, BAUD_RATE)
ser.flush()

# Initialize PiCamera
picam2 = Picamera2()
picam2.start()

# Frame processing variables
CENTER_X = 640 // 2  # Adjust according to your camera resolution
CENTER_Y = 480 // 2

# Uncomment if using GPIO for additional hardware
# GPIO_PIN = 18
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(GPIO_PIN, GPIO.OUT)

# Distance sensor example (uncomment if used)
# ultrasonic = DistanceSensor(echo=17, trigger=4)

# --- Main Loop ---
try:
    while True:
        # Capture frame from PiCamera
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply binary threshold and invert
        _, threshold = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
        inverted_threshold = cv2.bitwise_not(threshold)

        # Find contours
        contours, _ = cv2.findContours(inverted_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_img = cv2.cvtColor(inverted_threshold, cv2.COLOR_GRAY2BGR)

        # Detect the largest contour
        largest_contour = None
        max_contour_area = 0
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area > 500 and contour_area > max_contour_area:
                largest_contour = contour
                max_contour_area = contour_area

        # Process the largest contour
        if largest_contour is not None:
            epsilon = 0.03 * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)
            rect = cv2.minAreaRect(approx)
            ((x, y), _, _) = rect
            rect_center_x = int(x)
            rect_center_y = int(y)

            distance_x = rect_center_x - CENTER_X

            # Calculate the center of mass
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                center_of_mass_x = int(M["m10"] / M["m00"])
                center_of_mass_y = int(M["m01"] / M["m00"])
            else:
                center_of_mass_x, center_of_mass_y = 0, 0

            # Determine direction
            margin_distance = 150
            if center_of_mass_x > CENTER_X + 2 * distance_x + margin_distance:
                direction_text = "Turn Left"
            elif center_of_mass_x < CENTER_X + 2 * distance_x - margin_distance:
                direction_text = "Turn Right"
            else:
                direction_text = "Center"

            # Draw the direction and contour details
            cv2.putText(contours_img, direction_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.circle(contours_img, (center_of_mass_x, center_of_mass_y), 5, (0, 255, 255), -1)
            cv2.drawContours(contours_img, [approx], 0, (255, 0, 0), 2)
            cv2.line(contours_img, (CENTER_X, CENTER_Y), (rect_center_x, rect_center_y), (255, 0, 255), 2)

            # Display distance information
            text_distance = f"Distance X : {distance_x}"
            cv2.putText(contours_img, text_distance, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # Send data to Arduino
            data = f"{distance_x},{direction_text}\n"
            ser.write(data.encode())

        # Display the frame
        cv2.imshow('Contours', contours_img)

        # Quit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program terminated by user.")

finally:
    # Cleanup
    picam2.close()
    cv2.destroyAllWindows()
    ser.close()
