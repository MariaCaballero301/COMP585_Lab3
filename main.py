#!/usr/bin/env pybricks-micropython

#LAB 3 MOVING AROUND AN OBSTACLE

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
import math

# Initialize EV3
ev3 = EV3Brick()

# Initialize motors
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize sensors
ultrasonic = UltrasonicSensor(Port.S3)
right_touch = TouchSensor(Port.S1)
left_touch = TouchSensor(Port.S4)
gyro = GyroSensor(Port.S2)

# CONSTANTS (mm, deg/s, ms)
WHEEL_DIAMETER = 55.5     # mm -> CALIBRATE
SPEED = 200               # motor speed in deg/s (~97 mm/s)
MIN_DIST = 100            # wall-following range
MAX_DIST = 120
FAR_DIST = 250            # wall lost -> outer corner
AWAY_DIST = 500           # must get this far from hit point first
HIT_RADIUS = 200          # "back at the hit point"
TIME_LIMIT = 1000000      # 100 s -> go home before 2 min

# Pose (global variables)
x = 0.0
y = 0.0
prev_left = 0
prev_right = 0

# FUNCTIONS
def wait_for_center_button():
    while Button.CENTER not in ev3.buttons.pressed():
        wait(10)

def bumped():
    return left_touch.pressed() or right_touch.pressed()

def update_pose():
    # No GPS -> dead reckoning: pose must be updated constantly
    global x, y, prev_left, prev_right
    left = left_motor.angle()
    right = right_motor.angle()
    dL = (left - prev_left) * math.pi * WHEEL_DIAMETER / 360
    dR = (right - prev_right) * math.pi * WHEEL_DIAMETER / 360
    prev_left = left
    prev_right = right
    d = (dL + dR) / 2
    theta = (90 - gyro.angle()) * math.pi / 180   # degrees -> radians
    x = x + d * math.cos(theta)
    y = y + d * math.sin(theta)

def stop():
    left_motor.brake()
    right_motor.brake()

def turn_right(deg):
    # deg > 0 -> clockwise (right), deg < 0 -> counter-clockwise (left)
    target = gyro.angle() + deg
    while abs(target - gyro.angle()) > 2:
        error = target - gyro.angle()
        # slow down near the target to avoid overshoot
        speed = max(60, min(SPEED, abs(error) * 4))
        if error > 0:
            left_motor.run(speed)
            right_motor.run(-speed)
        else:
            left_motor.run(-speed)
            right_motor.run(speed)
        wait(5)
    stop()

def drive_distance(dist_mm, speed):
    start = (left_motor.angle() + right_motor.angle()) / 2
    target_deg = dist_mm * 360 / (math.pi * WHEEL_DIAMETER)
    while abs((left_motor.angle() + right_motor.angle()) / 2 - start) < target_deg:
        left_motor.run(speed)
        right_motor.run(speed)
        update_pose()
        wait(10)
    stop()

# MAIN CODE
wait_for_center_button()          # press first, then stay still for the reset
wait(500)                         # let the robot settle after pressing the button
gyro.reset_angle(0)
left_motor.reset_angle(0)
right_motor.reset_angle(0)
prev_left = 0
prev_right = 0
x = 0.0
y = 0.0
away = False
timer = StopWatch()

# 1. Go toward the obstacle
while not bumped():
    left_motor.run(SPEED)
    right_motor.run(SPEED)
    update_pose()
    wait(10)
stop()
update_pose()
ev3.speaker.beep()
hit_x = x
hit_y = y

drive_distance(30, -SPEED)        # back up a bit so the bumper doesn't scrape while turning
turn_right(90)
update_pose()                     # syncs prev_left/prev_right after the turn

# 2. Follow the wall
TARGET = (MIN_DIST + MAX_DIST) / 2      # 110 mm, ideal distance to the wall
KP = 1.5                            # proportional gain

while True:
    update_pose()
    dist_hit = math.sqrt((x - hit_x) ** 2 + (y - hit_y) ** 2)
    
    if dist_hit > AWAY_DIST:
        away = True

    # Condición de llegada al hit point con seguridad de tiempo
    if (away and dist_hit < HIT_RADIUS) or timer.time() > TIME_LIMIT:
        break
        
    if bumped():
        stop()
        drive_distance(40, -SPEED)
        update_pose()
        turn_right(60)
        update_pose()
        drive_distance(40, SPEED)
        update_pose()
    else:
        u = ultrasonic.distance()
        if u > FAR_DIST:
            # wall lost -> outer corner: sharp curve left
            left_motor.run(SPEED * 0.3)
            right_motor.run(SPEED)
        else:
            # P-control: the further from TARGET, the stronger the correction
            error = u - TARGET                              # >0 too far -> steer left
            turn = KP * error
            turn = max(-SPEED * 0.5, min(SPEED * 0.5, turn))  # limit the correction
            left_motor.run(SPEED - turn)
            right_motor.run(SPEED + turn)
    wait(10)
  
# Return to the start point
stop()
update_pose()
ev3.speaker.beep()

return_dist = math.sqrt(x ** 2 + y ** 2)
return_angle = math.atan2(-y, -x) * 180 / math.pi   # math convention (CCW +)

current_angle = 90 - gyro.angle()                     # same convention
diff = return_angle - current_angle                   # CCW positive
while diff > 180:
    diff -= 360
while diff < -180:
    diff += 360

turn_right(-diff)     # turn_right uses CW positive, so flip the sign
update_pose()

# Drive home holding the heading with the gyro
target_gyro = gyro.angle()
start = (left_motor.angle() + right_motor.angle()) / 2
target_deg = return_dist * 360 / (math.pi * WHEEL_DIAMETER)
while abs((left_motor.angle() + right_motor.angle()) / 2 - start) < target_deg:
    if bumped():
        break
    correction = (gyro.angle() - target_gyro) * 5     # drifted right -> steer left
    left_motor.run(SPEED - correction)
    right_motor.run(SPEED + correction)
    update_pose()
    wait(10)

stop()
ev3.speaker.beep()
