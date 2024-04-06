#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge
import cv2
import socket
import threading

# Server's IP address and port
HOST = '134.84.218.128'
PORT = 8554

# Delimiter to mark the end of each image
DELIMITER = b'\x00\xFF\x00\xFF'

# Function to receive commands from the server
def receive_commands(client_socket):
    try:
        while True:
            data = client_socket.recv(1024)
            # Process received data as needed
            print("Received command:", data.decode('utf-8'))
    except Exception as e:
        print(e)

# Callback function to handle incoming image messages
def image_callback(msg, client_socket):
    try:
        # Convert ROS Image message to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Encode the image as JPEG before sending
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, img_encoded = cv2.imencode('.jpg', cv_image, encode_param)

        print("Sending camera data")
        # Send the image data over TCP connection with delimiter
        client_socket.sendall(img_encoded.tobytes() + DELIMITER)
    except Exception as e:
        print(e)

def main():
    # Initialize ROS node
    rospy.init_node("image_subscriber", anonymous=True)

    # Create a TCP client socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        # Connect to the server
        client_socket.connect((HOST, PORT))

        # Start a separate thread to receive commands from the server
        receive_thread = threading.Thread(target=receive_commands, args=(client_socket,))
        receive_thread.start()

        # Subscribe to the image topic
        rospy.Subscriber("/camera/color/image_raw", Image, image_callback, client_socket)

        # Spin ROS node to process incoming messages
        rospy.spin()

if __name__ == "__main__":
    main()

