from socket import *
from golfbot.utils.linalg import Vector2
import time

class Controller:
    def __init__(self, robot_host: tuple[str, int], esp32_host: tuple[str,int]):
        self.robotHost = robot_host
        self.esp32Host = esp32_host
        self.robotSocket = socket(AF_INET, SOCK_DGRAM)
        self.esp32Socket = socket()

    def __del__(self):
        self.robotSocket.close()
        self.esp32Socket.close()

    def connect(self):
        self.esp32Socket.connect(self.esp32Host)

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

    def move_dir(self, direction: Vector2, not_backwards: bool = True):
        #print("i was here")
        print(direction)
        #if direction[1] > 0.2 and not_backwards:
        #print("i was also here")
        direction = Vector2(float(-direction[0]), float(direction[1]))
        command = f"[{direction.x:.3},{direction.y:.3}]".encode("UTF-8")
        self.robotSocket.sendto(command, self.robotHost)
        time.sleep(0.08)
        """else:
            direction = Vector2(-direction[0], direction[1])
            if direction[0] < 0:
                direction = Vector2(0.5, 0.0)
            else:
                direction = Vector2(-0.5, 0.0)
            #print(direction)
            command = f"[{direction.x:.3},{direction.y:.3}]".encode("UTF-8")
            self.robotSocket.sendto(command, self.robotHost)
            time.sleep(0.08)"""


    def turn_off_fan(self):
        self.robotSocket.sendto(b"h", self.robotHost)
        self.esp32Socket.send(b"OFF")

    def turn_on_fan(self):
        print("\n\n\nI am in the turn on function\n\n\n")
        self.esp32Socket.send("ON".encode("UTF-8"))
        self.robotSocket.sendto(b"h", self.robotHost)

    def open_door(self):
        for i in range(10):
            self.robotSocket.sendto(b"o", self.robotHost)
            time.sleep(0.02)

    def close_door(self):
        for i in range(10):
            self.robotSocket.sendto(b"p", self.robotHost)
            time.sleep(0.02)