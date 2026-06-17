from socket import *
from utils.Linalg.vector import Vector2
import time

class Controller:
    def __init__(self, robot_host: tuple[str, int], esp32_host: tuple[str,int]):
        self.robotHost = robot_host
        self.esp32Host = esp32_host
        self.robotSocket = socket(AF_INET, SOCK_DGRAM)
        self.esp32Socket = socket()
        self.esp32Socket.connect(esp32_host)

    def move(self, direction: str):
        if direction == 'w':
            self.robotSocket.sendto(b'w', self.robotHost)
        elif direction == 's':
            self.robotSocket.sendto(b's', self.robotHost)
        elif direction == 'w+a':
            self.robotSocket.sendto(b'wa', self.robotHost)
        elif direction == 'w+d':
            self.robotSocket.sendto(b'wd', self.robotHost)
        elif direction == 'a':
            self.robotSocket.sendto(b'a', self.robotHost)
        elif direction == 'd':
            self.robotSocket.sendto(b'd', self.robotHost)
        elif direction == 'o':
            self.robotSocket.sendto(b'o', self.robotHost)
        elif direction == 'p':
            self.robotSocket.sendto(b'p', self.robotHost)
        elif direction =='x':
            self.robotSocket.sendto(b'x', self.robotHost)
        else:
            self.robotSocket.sendto(b'h', self.robotHost)

    def move_dir(self, direction: Vector2, notbackwards: bool = True):
        if direction[1] < 0.2 and notbackwards:
            command = f"[{direction.x:.3},{direction.y:.3}]".encode("UTF-8")
            self.robotSocket.sendto(command, self.robotHost)
            time.sleep(0.02)
        else:
            if direction[0] < 0:
                direction = Vector2(0, -1)
            else:
                direction = Vector2(0, 1)
            command = f"[{direction.x:.3},{direction.y:.3}]".encode("UTF-8")
            self.robotSocket.sendto(command, self.robotHost)
            time.sleep(0.02)


    def turn_off_fan(self):
        self.esp32Socket.send(b"OFF")

    def turn_on_fan(self):
        self.esp32Socket.send(b"ON")

    def open_door(self):
        self.esp32Socket.send(b"o")
        time.sleep(0.02)

    def close_door(self):
        self.esp32Socket.send(b"p")
        time.sleep(0.02)