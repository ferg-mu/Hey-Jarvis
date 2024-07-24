import cv2
import numpy as np
import math
import paho.mqtt.client as paho
import time

# MQTT Topics
TOPIC_CONTROL = "/robot_control"
TOPIC_BRICK = "/brick"
TOPIC_SPEAK_POS = "/speak_pos"
TOPIC_SPEAK_SPEECH = "/speech"
TOPIC_SPEAK = "/speak"
TOPIC_ROBOT_PLACED = "/robot_placed"
TOPIC_HUMAN_PLACED = "/human_placed"
# Global variable to store the message payload
last_message = "None"
brick = "None"
previous_brick = 0
count = 0
once = True
wait = 10/0.02 #10 seconds

cap = cv2.VideoCapture(2)  # Capture video from the webcam

# Define color thresholds for green
lower_green = np.array([36, 50, 70])
upper_green = np.array([89, 255, 255])

def load_calibration_data(file_path):
    with np.load(file_path) as data:
        camera_matrix = data['camera_matrix']
        dist_coeffs = data['dist_coeffs']
    return camera_matrix, dist_coeffs

calibration_file_path = r'C:\Users\fearg\OneDrive\Desktop\iaac_code\workshop_iii-ii\calibration_data.npz'
camera_matrix, dist_coeffs = load_calibration_data(calibration_file_path)

# Callbacks for MQTT
def on_connect(client, userdata, flags, rc, properties=None):
    print(f"CONNACK received with code {rc}.")

def on_publish(client, userdata, mid, properties=None):
    print(f"mid: {mid}")

def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print(f"Subscribed: {mid} {granted_qos}")

def on_message(client, userdata, msg):
    global last_message, topic
    last_message = msg.payload.decode('utf-8')
    topic = msg.topic
    print(f"{msg.topic} {msg.qos} {last_message}")

# Initialize MQTT client
client = paho.Client(client_id="", protocol=paho.MQTTv5)
client.on_connect = on_connect
client.on_publish = on_publish
client.on_subscribe = on_subscribe
client.on_message = on_message
client.connect("mqtt-dashboard.com", 1883)

def calculate_angle(mid1, mid2):
    delta_y = mid2[1] - mid1[1]
    delta_x = mid2[0] - mid1[0]
    angle = math.atan2(delta_y, delta_x) * 180 / math.pi  # Convert radians to degrees
    return angle

def check_alignment_with_square(angle, square_angle):
    # Check if the angle of the center line is within a certain range of the square's edges
    angle_diff = abs(angle - square_angle)
    if angle_diff < 10 or angle_diff > 170:  # Example threshold for alignment
        return 'Aligned'
    else:
        return 'Not Aligned'

def detect_blocks_and_alignment(frame, active_zone):
    global count, once

    zones = {}  # Initialize zones dictionary
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Apply thresholding to handle different lighting conditions
    mask = cv2.inRange(hsv, (0, 0, 0), (179, 50, 50))
    hsv[mask > 0] = (0, 0, 0)
    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)
    # Define region of interest (ROI)
    height, width = mask.shape
    roi_top_left = (int(width * 0.2), int(height * 0.2))
    roi_bottom_right = (int(width * 0.8), int(height * 0.8))
    # Draw a red rectangle around the ROI
    cv2.rectangle(frame, roi_top_left, roi_bottom_right, (0, 0, 255), 2)

    # Define the projected square in the center of the frame
    square_size = 100  # Example size of the square
    square_top_left = (width // 2 - square_size // 2, height // 2 - square_size // 2)
    square_bottom_right = (width // 2 + square_size // 2, height // 2 + square_size // 2)
    square_top_right = (square_bottom_right[0], square_top_left[1])
    square_bottom_left = (square_top_left[0], square_bottom_right[1])
    square_edges = [
        (square_top_left, square_top_right),    # Edge 0
        (square_bottom_right, square_bottom_left),# Edge 2
        (square_top_right, square_bottom_right),# Edge 1
        (square_bottom_left, square_top_left)   # Edge 3
    ]
    # Label the edges
    for i, (start, end) in enumerate(square_edges):
        mid = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
        cv2.putText(frame, f'{i}', mid, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    
    cv2.rectangle(frame, square_top_left, square_bottom_right, (255, 0, 0), 2)

    # Create orange rectangles based on each edge of the projected square
    offset = 30
    for i, (edge_start, edge_end) in enumerate(square_edges):
        if i == 0:  # Top edge
            rectangle_top_left = (edge_start[0], edge_start[1])
            rectangle_bottom_right = (edge_end[0], edge_end[1] + offset)
        elif i == 2:  # Right edge
            rectangle_top_left = (edge_start[0] - offset, edge_start[1])
            rectangle_bottom_right = (edge_end[0], edge_end[1])
        elif i == 1:  # Bottom edge
            rectangle_top_left = (edge_start[0], edge_start[1] - offset)
            rectangle_bottom_right = (edge_end[0], edge_end[1])
        elif i == 3:  # Left edge
            rectangle_top_left = (edge_start[0], edge_start[1])
            rectangle_bottom_right = (edge_end[0] + offset, edge_end[1])
        
        # Highlight the active zone with a different color
        color = (0, 255, 0) if active_zone == f'zone_{i}' else (0, 165, 255)
        
        # Draw the rectangle on the frame
        cv2.rectangle(frame, rectangle_top_left, rectangle_bottom_right, color, 2)

        # Append this rectangle to the zones dictionary
        zones[f'zone_{i}'] = (rectangle_top_left, rectangle_bottom_right, i)

    # Mask everything outside the ROI
    roi_mask = np.zeros_like(mask)
    roi_mask[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]] = mask[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]
    # Find contours
    contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables for the longest line
    longest_length = 0
    longest_mid1 = None
    longest_mid2 = None
    longest_box = None

    # Draw contours and bounding rectangles on the frame
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # Minimum area to filter noise
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # Calculate the midpoints of the shorter sides of the bounding box
            (x1, y1), (x2, y2), (x3, y3), (x4, y4) = box
            # Determine which sides are the shorter sides
            d1 = np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))
            d2 = np.linalg.norm(np.array([x2, y2]) - np.array([x3, y3]))
            if d1 < d2:
                mid1 = ((x1 + x2) // 2, (y1 + y2) // 2)
                mid2 = ((x3 + x4) // 2, (y3 + y4) // 2)
            else:
                mid1 = ((x2 + x3) // 2, (y2 + y3) // 2)
                mid2 = ((x1 + x4) // 2, (y1 + y4) // 2)
            length = np.linalg.norm(np.array(mid1) - np.array(mid2))

            # Check if this line is the longest one found
            if length > longest_length:
                longest_length = length
                longest_mid1 = mid1
                longest_mid2 = mid2
                longest_box = box

    if longest_mid1 is not None and longest_mid2 is not None:
        # Draw the longest line and its bounding box
        cv2.drawContours(frame, [longest_box], 0, (0, 255, 0), 2)
        cv2.line(frame, longest_mid1, longest_mid2, (255, 0, 0), 2)
        cv2.circle(frame, longest_mid1, 5, (255, 0, 0), -1)
        cv2.circle(frame, longest_mid2, 5, (255, 0, 0), -1)
        cv2.putText(frame, f'Length: {longest_length:.2f}', longest_mid1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # Skip lines that are equal to or less than 53mm
        if longest_length > 56:
            # Calculate the midpoint of the center line
            midpoint = ((longest_mid1[0] + longest_mid2[0]) // 2, (longest_mid1[1] + longest_mid2[1]) // 2)
            cv2.circle(frame, midpoint, 5, (0, 0, 255), -1)

            alignment_status = "Not Aligned"
            # Check if both the start and end points are inside the active zone
            zone_top_left, zone_bottom_right, edge_index = zones[active_zone]
            
            if ((zone_top_left[0] <= longest_mid1[0] <= zone_bottom_right[0] and
                zone_top_left[1] <= longest_mid1[1] <= zone_bottom_right[1] and
                zone_top_left[0] <= longest_mid2[0] <= zone_bottom_right[0] and
                zone_top_left[1] <= longest_mid2[1] <= zone_bottom_right[1]) or 
                (zone_top_left[0] >= longest_mid1[0] >= zone_bottom_right[0] and
                zone_top_left[1] <= longest_mid1[1] <= zone_bottom_right[1] and
                zone_top_left[0] >= longest_mid2[0] >= zone_bottom_right[0] and
                zone_top_left[1] <= longest_mid2[1] <= zone_bottom_right[1]) or
                (zone_top_left[0] <= longest_mid1[0] <= zone_bottom_right[0] and
                zone_top_left[1] >= longest_mid1[1] >= zone_bottom_right[1] and
                zone_top_left[0] <= longest_mid2[0] <= zone_bottom_right[0] and
                zone_top_left[1] >= longest_mid2[1] >= zone_bottom_right[1])):
                # Calculate and display the angle of the center line
                angle = calculate_angle(longest_mid1, longest_mid2)
                # Check alignment with the respective edge of the zone
                if edge_index in [0, 1]:  # Horizontal edges
                    zone_angle = 0  # Horizontal alignment
                else:  # Vertical edges
                    zone_angle = 90  # Vertical alignment
                alignment_status = check_alignment_with_square(angle, zone_angle)
            
            cv2.putText(frame, alignment_status, (longest_mid1[0], longest_mid1[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            if alignment_status == "Aligned":
                time.sleep(2.0)
                alignment_status = check_alignment_with_square(angle, zone_angle)
                if alignment_status == "Aligned":
                    client.publish(TOPIC_SPEAK_SPEECH, payload="Its aligned perfectly;", qos=0)
                    client.publish(TOPIC_SPEAK_POS, payload="True", qos=0)
                    client.publish(TOPIC_SPEAK, payload="True", qos=0)
                    time.sleep(3.0)
            elif count == wait and once == True:
                client.publish(TOPIC_SPEAK_SPEECH, payload="The piece is not where it should, please place it properly:", qos=0)
                client.publish(TOPIC_SPEAK_POS, payload="True", qos=0)
                client.publish(TOPIC_SPEAK, payload="True", qos=0)
                once = False

            time.sleep(0.02)
            count += 1

    return frame, zones

def detect_aruco_markers(frame, camera_matrix, dist_coeffs):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        for i in range(len(ids)):
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec[i], tvec[i], 0.05)
        for idss, cornerss in zip(ids, corners):
            cv2.polylines(frame, [cornerss.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)
            cornerss = cornerss.reshape(4, 2)
            cornerss = cornerss.astype(int)
            top_right = cornerss[0].ravel()
            cv2.putText(frame, f"id: {idss[0]}", top_right, cv2.FONT_HERSHEY_PLAIN, 1.3, (200, 100, 0), 2, cv2.LINE_AA)

    return frame

def main():
    global last_message, brick, count, once, previous_brick
    # Subscribe to MQTT topic and start the loop
    client.subscribe(TOPIC_BRICK, qos=0)
    client.loop_start()

    while True:
        if last_message != "None":
            brick = int(last_message)
            active_zone = "zone_" + str(brick)  # Example: specify the active zone here

            if brick != previous_brick:
                count = 0
                once = True
                previous_brick = brick

            print(active_zone)

            ret, frame = cap.read()
            if not ret:
                break

            # Detect ArUco markers
            frame = detect_aruco_markers(frame, camera_matrix, dist_coeffs)
            
            # Detect blocks and analyze alignment
            frame, zone = detect_blocks_and_alignment(frame, active_zone)

            # Increase the size of the frame by 1.5 times for better visualization
            frame = cv2.resize(frame, (int(frame.shape[1] * 1.5), int(frame.shape[0] * 1.5)))

            # Display the processed frame
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
