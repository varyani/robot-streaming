import asyncio
import cv2
import numpy as np
from sensor_msgs.msg import Joy
import json
import websockets

HOST = '0.0.0.0'
PORT = 8554
DELIMITER = b'\x00\xFF\x00\xFF'  # Delimiter to mark the end of each image

async def send_commands(websocket, path):
    try:
        while True:
            msg = await websocket.recv()
            print("Received joystick data:", msg)
            # Process received joystick data as needed
    except Exception as e:
        print(e)

async def image_receiver(websocket, path):
    count = 0
    buffer = b''
    while True:
        data = await websocket.recv()
        buffer += data
        while DELIMITER in buffer:
            print("image number", count)
            count += 1
            img_data, _, buffer = buffer.partition(DELIMITER)
            nparr = np.frombuffer(img_data, np.uint8)
            img_np = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            cv2.imshow('Received Image', img_np)
            cv2.waitKey(1)
            print("Image data received and displayed.")

async def main():
    # Initialize ROS node
    rospy.init_node('image_receiver')

    # Start WebSocket server
    async with websockets.serve(image_receiver, HOST, PORT):
        print(f"Server is listening for connections on ws://{HOST}:{PORT}...")

        # Run the event loop to process WebSocket connections
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())

