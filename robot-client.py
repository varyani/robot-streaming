#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket

#HOST = '15.181.49.20'  # Server's IP address
HOST = '134.84.218.128'  # Server's IP address
#PORT = 65432        # Port to connect to
PORT = 8554        # Port to connect to
DELIMITER = b'\x00\xFF\x00\xFF'  # Delimiter to mark the end of each image

def image_callback(msg, client_socket):
    try:
        # Convert ROS Image message to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Encode the image as JPEG before sending
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, img_encoded = cv2.imencode('.jpg', cv_image, encode_param)

        print("sending camera data")
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

        # Subscribe to the image topic
        rospy.Subscriber("/camera/color/image_raw", Image, image_callback, client_socket)

        # Spin ROS node to process incoming messages
        rospy.spin()

if __name__ == "__main__":
    main()

