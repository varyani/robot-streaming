#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge
import cv2
import asyncio
import websockets

# Server's IP address and port
HOST = '134.84.218.128'
PORT = 8554

# Delimiter to mark the end of each image
DELIMITER = b'\x00\xFF\x00\xFF'

# Function to receive commands from the server
async def receive_commands(websocket):
    try:
        async for data in websocket:
            # Process received data as needed
            print("Received command:", data)
    except Exception as e:
        print(e)

# Callback function to handle incoming image messages
async def image_callback(msg, websocket):
    try:
        # Convert ROS Image message to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Encode the image as JPEG before sending
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, img_encoded = cv2.imencode('.jpg', cv_image, encode_param)

        print("Sending camera data")
        # Send the image data over WebSocket connection with delimiter
        await websocket.send(img_encoded.tobytes() + DELIMITER)
    except Exception as e:
        print(e)

async def main():
    # Initialize ROS node
    rospy.init_node("image_subscriber", anonymous=True)

    # Connect to WebSocket server
    async with websockets.connect(f"ws://{HOST}:{PORT}") as websocket:
        # Start a separate coroutine to receive commands from the server
        receive_task = asyncio.create_task(receive_commands(websocket))

        # Subscribe to the image topic
        rospy.Subscriber("/camera/color/image_raw", Image, lambda msg: asyncio.create_task(image_callback(msg, websocket)))

        # Spin ROS node to process incoming messages
        await rospy.spin()

if __name__ == "__main__":
    asyncio.run(main())

