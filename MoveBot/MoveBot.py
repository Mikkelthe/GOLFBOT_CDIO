from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, MoveTank
from time import sleep

# Motors connected to ports B and C
tank = MoveTank(OUTPUT_B, OUTPUT_C)

def move_backward(distance):
    tank.on_for_seconds(-100,-100, distance/(51.5/2))
    return
def move_forward(distance):
    tank.on_for_seconds(100,100, distance/(51.5/2))
    return
    
def turn(degrees,direction):
    if direction == "left":
        tank.on_for_seconds(50, -50, (degrees/360)*4.83)
    else:
        tank.on_for_seconds(-50, 50, (degrees/360)*4.83)
    return
# Move forward at 100% speed
#tank.on(100, 100)
#sleep(4)

# Stop
#tank.off()

# Turning
for (i) in range(4):
    move_forward(100)
    move_backward(100)

for (i) in range(50):
    turn(90, "left")
    turn(90, "right")
turn(180, "left")
turn(180, "right")
turn(360, "left")
turn(360, "right")


# 720 degrees ≈ 2 full wheel rotations.
#tank.on_for_degrees(50, 50, 720)