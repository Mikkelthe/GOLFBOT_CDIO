from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, MoveTank
from time import sleep

# Motors connected to ports B and C
tank = MoveTank(OUTPUT_B, OUTPUT_C)

# Move forward at 50% speed
tank.on(50, 50)
sleep(2)

# Stop
tank.off()

# Turning
tank.on_for_seconds(50, -50, 1)

# 720 degrees ≈ 2 full wheel rotations.
tank.on_for_degrees(50, 50, 720)