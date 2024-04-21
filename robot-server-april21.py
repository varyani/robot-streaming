import cv2
import socket
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import json
import time
import csv
import pickle
import sys

HOST = '0.0.0.0'     
PORT = 8554       
DELIMITER = b'\x00\xFF\x00\xFF'  # Delimiter to mark the end of each image
CSV_FILE = 'server_timestamp.csv'

def send_commands(msg):
    global conn
    try:
        scaling_z = 2
        scaling_x = 1.3
        cmd = Twist()
        if (1 + msg.axes[3]):
            cmd.angular.z = 0
            cmd.linear.x = 0
        else:
            cmd.angular.z = scaling_z*(msg.axes[0])
            cmd.linear.x = scaling_x*(1 + msg.axes[2])    
            # Create a dictionary containing cmd.linear.x and cmd.angular.z values
            cmd_dict = {'linear_x': cmd.linear.x, 'angular_z': cmd.angular.z}
            # Serialize the dictionary into JSON format
            cmd_json = json.dumps(cmd_dict)
            # Send the JSON data over the socket connection with a delimiter
            conn.sendall(cmd_json.encode() + b'\n')  # Adding a newline as a delimiter
            print(cmd)
    except Exception as e:
        print(e)

def main():
    global conn                     
    rospy.init_node('image_receiver')
    with open(CSV_FILE, mode ='a') as file:
        writer = csv.writer(file)
        writer.writerow(['Video Frame Number', 'Transmission Beginning Time', 'Server Receive Time', 'One-way network delay (seconds)', 'Video Frame Processing Delay (seconds)','Frame size'])

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))
        server_socket.listen()

        print("Server is listening for connections...")
        conn, addr = server_socket.accept()
        count = 1
        print(f"Connected by {addr}")

        buffer = b''
        rospy.Subscriber("joy", Joy, send_commands)
        while not rospy.is_shutdown():
            data = conn.recv(1024)
            if not data:
                break
            buffer += data
            while DELIMITER in buffer:
                server_receive_time=time.time()

                img_data_with_time, _, buffer = buffer.partition(DELIMITER)
                transmission_beginning_time, img_data = pickle.loads(img_data_with_time)
                network_delay=server_receive_time-transmission_beginning_time
                # Calculate the size of img_data
                img_data_size = sys.getsizeof(img_data)*8 # bits
    

                nparr = np.frombuffer(img_data, np.uint8)
                img_np = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                cv2.imshow('Received Image', img_np)

                cv2.waitKey(1)  
                image_display_time=time.time()
                frame_processing_delay=image_display_time-server_receive_time
                with open(CSV_FILE, mode ='a') as file:
                    writer = csv.writer(file)
                    writer.writerow([count,transmission_beginning_time, server_receive_time, network_delay , frame_processing_delay, img_data_size])
                count += 1
                    
            # If 'q' is pressed, exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

