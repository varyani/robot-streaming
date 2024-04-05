import cv2
import socket
import numpy as np

HOST = '0.0.0.0'     
PORT = 8554       
DELIMITER = b'\x00\xFF\x00\xFF'  # Delimiter to mark the end of each image

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))
        server_socket.listen()

        print("Server is listening for connections...")
        conn, addr = server_socket.accept()
        with conn:
            print(f"Connected by {addr}")

            buffer = b''
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                buffer += data
                while DELIMITER in buffer:
                    img_data, _, buffer = buffer.partition(DELIMITER)
                    nparr = np.frombuffer(img_data, np.uint8)
                    img_np = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    cv2.imshow('Received Image', img_np)
                    cv2.waitKey(1)  
                    print("Image data received and displayed.")
                    
                # If 'q' is pressed, exit the loop
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

if __name__ == "__main__":
    main()
