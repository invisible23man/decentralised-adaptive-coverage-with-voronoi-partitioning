# main.py
import subprocess
import threading
import socket
import struct
import queue

def listen_for_messages(message_queue):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('', 12345))  # Listen for UDP packets on port 12345
        while True:
            data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
            time_taken = struct.unpack('!d', data)[0]  # Unpack the time taken from the binary format
            message_queue.put((time_taken, addr))

radii = [1, 2, 3, 4, 5]  # Radii for the circles

processes = []
for radius in radii:
    process = subprocess.Popen(["python", "./tests/plot_circle.py", str(radius)])
    processes.append(process)

message_queue = queue.Queue()

listener_thread = threading.Thread(target=listen_for_messages, args=(message_queue,))
listener_thread.start()

for process in processes:
    process.wait()

listener_thread.join()

while not message_queue.empty():
    time_taken, addr = message_queue.get()
    print(f"Received time {time_taken} from {addr}")
