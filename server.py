#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from golfbot import GolfBot
from socket import *

HOST = '0.0.0.0'
PORT = 6853


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
bot = GolfBot(EV3Brick(), Motor(Port.B), Motor(Port.C), Motor(Port.A))

def command_udp():
    s = socket(AF_INET, SOCK_DGRAM)
    s.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    print("Binding socket")
    s.bind((HOST, PORT))
    move_amount = 10
    turn_amount = 10
    while True:
        data, address = s.recvfrom(32)
        inpt = data.decode('utf-8')
        if inpt == 'w':
            bot.move_fwd()
        elif inpt == 's':
            bot.move_back()
        elif inpt == 'wa':
            bot.move_fwd_left()
        elif inpt == 'wa':
            bot.move_fwd_right()
        elif inpt == 'a':
            bot.turn_left()
        elif inpt == 'd':
            bot.turn_right()
        elif inpt == 'o':
            bot.open_ramp()
        elif inpt == 'p':
            bot.close_ramp()
        elif inpt == 'x':
            break
        else:
            bot.stop()
    s.close()


def command_tcp():
    s = socket(AF_INET, SOCK_STREAM)
    s.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    print("Binding socket")
    s.bind((HOST, PORT))
    print("Listening for connections")
    s.listen(2)
    conn, addr = s.accept()
    print("Accepting connection to client")
    while True:
        data = conn.recv(32)
        if not data:
            break
        inpt = data.decode('utf-8')
        print(inpt)
        command, value = inpt.split(' ')

        if command == 'fwd':
            bot.move(int(value))
        elif command == "back":
            bot.move(-int(value))
        elif command == 'left':
            bot.turn(-int(value))
        elif command == 'right':
            bot.turn(int(value))
        elif command == 'stop':
            break
        conn.send(b'ack')        
    conn.close()
    s.close()

command_udp()
