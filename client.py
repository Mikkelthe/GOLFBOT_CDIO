import keyboard
from socket import *
import time 
import math

HOST = '172.20.10.7'  # The server's hostname or IP address
PORT = 6853  # The port used by the server

def client_udp():
    last_command = b"_"
    with socket(AF_INET, SOCK_DGRAM) as s:
        while True:
            x, y = 0,0
            if keyboard.is_pressed("w"):
                y += 1.0
            if keyboard.is_pressed("s"):
                y -= 1.0
            if keyboard.is_pressed("a"):
                x -= 1.0
            if keyboard.is_pressed("d"):
                x += 1.0
            
            if(x**2 + y**2 > 1):
                magnitude = math.sqrt(x**2 + y**2)
                x /= magnitude
                y /= magnitude

            if keyboard.is_pressed('space'):
                x *= 2
                y *= 2
            
            command = b"h"
            
            if keyboard.is_pressed('o'):
                command = b"o"
            elif keyboard.is_pressed('p'):
                command = b"p"
            elif keyboard.is_pressed('x'):
                s.sendto(b'x', (HOST, PORT))
                return
            elif x == 0 and y == 0:
                command = b"h"
            else:
                command = f"[{x:.3f}, {y:.3f}]".encode("UTF-8")
            if command != last_command: # Ensure that we stop the robot before executing the next command
                s.sendto(b"h", (HOST, PORT))
                time.sleep(0.02)
            s.sendto(command, (HOST, PORT))
            last_command = command
            time.sleep(0.02)
            
def client_tcp():
    with socket(AF_INET, SOCK_STREAM) as s:
        print("Connecting...")
        s.connect((HOST, PORT))
        print("Finished Connecting")
        while True:
            print("Input: ", end="")
            key = keyboard.read_key()
            print(f"Key: {key}")
            if key == 'w':
                s.send(b'fwd 10')
            elif key == 's':
                s.send(b'back 10')
            elif key == 'a':
                s.send(b'left 10')
            elif key == 'd':
                s.send(b'right 10')
            elif key == 'x':
                break
            print()
            s.recv(32)
        
        s.send(b'stop')

client_udp()