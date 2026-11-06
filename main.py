
#LAB 3 MOVING AROUND AN OBSTICLE

#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize EV3
ev3 = EV3Brick()

# Initialize motors
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize sensors
ultrasonic = UltrasonicSensor(Port.S3)
right_touch = TouchSensor(Port.S1)
left_touch = TouchSensor(Port.S2)

# Initialize drive base
robot = DriveBase(left_motor, right_motor,
                  wheel_diameter=55.5, axle_track=104)

# Robot settings
robot.settings(straight_speed=100)

# Assignment distances (mm)
forward_distance = 1400
wall_distance = 400
reverse_distance = 400

# Sensor placement adjustments (mm)
ultrasonic_offset = 20
reverse_offset = 55

# 1. Move forward 1.4 meters
robot.straight(forward_distance)

# 2. Beep until center button is pressed
while Button.CENTER not in ev3.buttons.pressed():
    ev3.speaker.beep()
    wait(100)

ev3.screen.clear()

# 3. Move until robot is 40 cm from wall
while True:
    robot.drive(100, 0)
    distance = ultrasonic.distance()
    ev3.screen.print(distance)

    if distance < wall_distance + ultrasonic_offset:
        robot.stop()

        while Button.CENTER not in ev3.buttons.pressed():
            ev3.speaker.beep()
            wait(100)

        break

    wait(100)

# 4. Move until touch sensor hits wall
while True:
    robot.drive(100, 0)

    if right_touch.pressed() or left_touch.pressed():
        robot.straight(-(reverse_distance - reverse_offset))
        break
