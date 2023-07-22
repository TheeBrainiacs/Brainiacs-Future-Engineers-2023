import cv2
import numpy as np
import serial
import time

# Known object height (in millimeters)
KNOWN_HEIGHT = 100

# Green color range in HSV
LOWER_BLUE1 = np.array([40, 100, 100])
UPPER_BLUE1 = np.array([70, 255, 255])
LOWER_BLUE2 = np.array([70, 100, 100])
UPPER_BLUE2 = np.array([100, 255, 255])

# Red color range in HSV
LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([179, 255, 255])

# Focal length and optical center (replace these values with your camera calibration)
f_x = 1430
f_y = 1450
c_x = 635
c_y = 60

# Configure the serial port
arduino_port = '/dev/ttyACM0'  # Replace with your Arduino's port
baud_rate = 9600

# Create a serial object
ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # Wait for Arduino to initialize

def find_distance_to_object(known_height, focal_length, perceived_height):
    return (known_height * focal_length) / perceived_height

def find_horizontal_angle(c_x, perceived_x, f_x):
    return np.arctan((perceived_x - c_x) / f_x)

def main():
    cap = cv2.VideoCapture(0)

    closest_blue_distance = float('inf')
    closest_blue_rect = None
    closest_red_distance = float('inf')
    closest_red_rect = None

    mangle = 0
    red_cst = 20  # Adjustment value for red object
    blue_cst = -20  # Adjustment value for blue object

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create masks for the blue and red color ranges
        mask_blue1 = cv2.inRange(hsv, LOWER_BLUE1, UPPER_BLUE1)
        mask_blue2 = cv2.inRange(hsv, LOWER_BLUE2, UPPER_BLUE2)
        mask_blue = cv2.bitwise_or(mask_blue1, mask_blue2)
        mask_red1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
        mask_red2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Apply morphological operations to clean up the masks
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        # Find contours in the masks
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Reset closest distances and rectangles
        closest_blue_distance = float('inf')
        closest_blue_rect = None
        closest_red_distance = float('inf')
        closest_red_rect = None

        for contour in contours_blue:
            # Filter out small contours (noise)
            area = cv2.contourArea(contour)
            if area < 1000:
                continue

            # Find the bounding rectangle for the contour
            x, y, width, height = cv2.boundingRect(contour)

            # Calculate the perceived height of the blue object
            perceived_height = height

            # Calculate the distance to the object
            distance = find_distance_to_object(KNOWN_HEIGHT, f_y, perceived_height)

            # Update closest blue object
            if distance < closest_blue_distance:
                closest_blue_distance = distance
                closest_blue_rect = (x, y, width, height)

        for contour in contours_red:
            # Filter out small contours (noise)
            area = cv2.contourArea(contour)
            if area < 1000:
                continue

            # Find the bounding rectangle for the contour
            x, y, width, height = cv2.boundingRect(contour)

            # Calculate the perceived height of the red object
            perceived_height = height

            # Calculate the distance to the object
            distance = find_distance_to_object(KNOWN_HEIGHT, f_y, perceived_height)

            # Update closest red object
            if distance < closest_red_distance:
                closest_red_distance = distance
                closest_red_rect = (x, y, width, height)

        # Draw bounding rectangles for closest objects and display the distance and angle
        if closest_blue_rect is not None:
            x, y, width, height = closest_blue_rect
            perceived_x = x + width / 2
            perceived_y = y + height / 2
            horizontal_angle = find_horizontal_angle(c_x, perceived_x, f_x)

            cv2.rectangle(frame, (x, y), (x + width, y + height), (255, 0, 0), 2)
            cv2.putText(frame, f"Blue - Distance: {closest_blue_distance:.2f} mm", (x, y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.putText(frame, f"Horizontal Angle: {np.degrees(horizontal_angle):.2f} degrees", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        if closest_red_rect is not None:
            x, y, width, height = closest_red_rect
            perceived_x = x + width / 2
            perceived_y = y + height / 2
            horizontal_angle1 = find_horizontal_angle(c_x, perceived_x, f_x)

            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 255), 2)
            cv2.putText(frame, f"Red - Distance: {closest_red_distance:.2f} mm", (x, y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, f"Horizontal Angle: {np.degrees(horizontal_angle1):.2f} degrees", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Save the angle to mangle variable
        if closest_red_rect is not None and closest_blue_rect is not None:
            if closest_red_distance < 600 and closest_blue_distance < 600:
                mangle = np.degrees(horizontal_angle) + blue_cst if closest_red_distance < closest_blue_distance else np.degrees(horizontal_angle1) + red_cst
            elif closest_red_distance < 600:
                mangle = np.degrees(horizontal_angle1) + red_cst
            elif closest_blue_distance < 600:
                mangle = np.degrees(horizontal_angle) + blue_cst
            else:
                mangle = 0
        elif closest_red_rect is not None:
            if closest_red_distance < 600:
                mangle = np.degrees(horizontal_angle1) + red_cst
            else:
                mangle = 0
        elif closest_blue_rect is not None:
            if closest_blue_distance < 600:
                mangle = np.degrees(horizontal_angle) + blue_cst
            else:
                mangle = 0
        else:
            mangle = 0

        # Send the horizontal angle to Arduino
        if mangle is not None:
            mangle = int(mangle)  # Convert to an integer before sending
            ser.write(str(mangle).encode())
            ser.write(b'\n')  # Optional: Send a newline character to indicate the end of the value

        # Rest of the code...

        # Print the mangle
        print(f"mangle: {mangle}")

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if _name_ == "_main_":
    main()