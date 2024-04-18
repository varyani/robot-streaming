#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import socket
import threading
import json
import time
import csv

# Server's IP address and port
HOST = '134.84.218.128'
PORT = 8554
CSV_FILE = 'timestamps.csv'
count = 0

# Delimiter to mark the end of each image
DELIMITER = b'\x00\xFF\x00\xFF'

# Function to receive commands from the server
def receive_commands(client_socket,cmd_vel_pub):
    try:
        buffer = b''  # Initialize buffer to store leftover data
        while True:
            # Read data from the socket connection
            recv_data = client_socket.recv(1024)
            if not recv_data:
                break
            # Append received data to buffer
            buffer += recv_data
            # Split buffer by newline character
            parts = buffer.split(b'\n')
            # Process complete JSON objects
            for part in parts[:-1]:  # Skip the last part as it may be incomplete
                # Decode received data as UTF-8 and remove trailing newline
                recv_data_str = part.decode('utf-8').strip()
                # Parse JSON object
                try:
                    cmd_dict = json.loads(recv_data_str)
                    # Extract cmd.linear.x and cmd.angular.z values from the dictionary
                    cmd_linear_x = cmd_dict.get('linear_x')
                    cmd_angular_z = cmd_dict.get('angular_z')
                    print("Received Linear.x:", cmd_linear_x)
                    print("Received Angular.z:", cmd_angular_z)
                    # Process received commands as needed
                    final_cmd=Twist()
                    final_cmd.linear.x=cmd_linear_x
                    final_cmd.angular.z=cmd_angular_z
                    cmd_vel_pub.publish(final_cmd)
                except json.JSONDecodeError as e:
                    print("Error decoding JSON:", e)
            # Update buffer with any leftover data after the last newline
            buffer = parts[-1]
    except Exception as e:
        print("Error receiving data:", e)

# Callback function to handle incoming image messages
def image_callback(msg, client_socket):
    try:
        # Convert ROS Image message to OpenCV format
        global count
        count = count +1
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Encode the image as JPEG before sending
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, img_encoded = cv2.imencode('.jpg', cv_image, encode_param)

        print("Sending camera data")
        # Send the image data over TCP connection with delimiter
        transmission_beginning_time=time.time()
        client_socket.sendall(img_encoded.tobytes() + DELIMITER)
        with open(CSV_FILE, mode ='a') as file:
            writer = csv.writer(file)
            writer.writerow([count,transmission_beginning_time]) 
    except Exception as e:
        print(e)

def main():
    # Initialize ROS node
    rospy.init_node("image_subscriber", anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel/managed',Twist,queue_size=10)
    # Create a TCP client socket
    with open(CSV_FILE, mode ='a') as file:
        writer = csv.writer(file)
        writer.writerow(['Video frame number','Transmission Beginning Time']) 
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        # Connect to the server
        client_socket.connect((HOST, PORT))

        # Start a separate thread to receive commands from the server
        receive_thread = threading.Thread(target=receive_commands, args=(client_socket,cmd_vel_pub,))
        receive_thread.start()

        # Subscribe to the image topic
        rospy.Subscriber("/camera/color/image_raw", Image, image_callback, client_socket)

        # Spin ROS node to process incoming messages
        rospy.spin()

if __name__ == "__main__":
    main()
