import keyboard
from socket import *
import time 

HOST = '172.20.10.7'  # The server's hostname or IP address
PORT = 6853  # The port used by the server

def client_udp():
    with socket(AF_INET, SOCK_DGRAM) as s:
        while True:
            key = keyboard.read_key()
            if keyboard.is_pressed('w'):
                s.sendto(b'w', (HOST, PORT))
            elif keyboard.is_pressed('s'):
                s.sendto(b's', (HOST, PORT))
            elif keyboard.is_pressed('w+a'):
                s.sendto(b'wa', (HOST, PORT))
            elif keyboard.is_pressed('w+d'):
                s.sendto(b'wd', (HOST, PORT))
            elif keyboard.is_pressed('a'):
                s.sendto(b'a', (HOST, PORT))
            elif keyboard.is_pressed('d'):
                s.sendto(b'd', (HOST, PORT))
            elif keyboard.is_pressed('o'):
                s.sendto(b'o', (HOST, PORT))
            elif keyboard.is_pressed('p'):
                s.sendto(b'p', (HOST, PORT))
            elif keyboard.is_pressed('x'):
                s.sendto(b'x', (HOST, PORT))
            else:
                s.sendto(b'h', (HOST, PORT))

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