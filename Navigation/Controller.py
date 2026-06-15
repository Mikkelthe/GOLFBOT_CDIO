from socket import *

class Controller:
    def __init__(self, robot_host: tuple[str, int], esp32_host: tuple[str,int]):
        self.robotHost = robot_host
        self.esp32Host = esp32_host
        self.robotSocket = socket(AF_INET, SOCK_DGRAM)
        self.esp32Socket = socket()
        self.esp32Socket.connect(esp32_host)

    def move(self, direction: str):
        if str == 'w':
            self.robotSocket.sendto(b'w', self.robotHost)
        elif str == 's':
            self.robotSocket.sendto(b's', self.robotHost)
        elif str == 'w+a':
            self.robotSocket.sendto(b'wa', self.robotHost)
        elif str == 'w+d':
            self.robotSocket.sendto(b'wd', self.robotHost)
        elif str == 'a':
            self.robotSocket.sendto(b'a', self.robotHost)
        elif str == 'd':
            self.robotSocket.sendto(b'd', self.robotHost)
        elif str == 'o':
            self.robotSocket.sendto(b'o', self.robotHost)
        elif str == 'p':
            self.robotSocket.sendto(b'p', self.robotHost)
        elif str =='x':
            self.robotSocket.sendto(b'x', self.robotHost)
        else:
            self.robotSocket.sendto(b'h', self.robotHost)

    def turn_off_fan(self):
        self.esp32Socket.send(b"OFF")

    def turn_on_fan(self):
        self.esp32Socket.send(b"ON")