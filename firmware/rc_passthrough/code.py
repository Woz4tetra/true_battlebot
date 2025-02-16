import os
import time
import pwmio
import board
import digitalio

import time
import board
import pwmio
from adafruit_motor import servo


def arange(*args):
    if len(args) == 1:
        start = 0.0
        stop = args[0]
        step = 1.0
    elif len(args) == 2:
        start = args[0]
        stop = args[1]
        step = 1.0
    elif len(args) == 3:
        start = args[0]
        stop = args[1]
        step = args[2]
    else:
        raise TypeError("arange expected at least 1 argument, got 0")
    value = start
    if start < stop:
        while value < stop:
            yield value
            value += step
    else:
        while stop < value:
            yield value
            value += step
        


# import wifi

# wifi.radio.connect(ssid=os.getenv("CIRCUITPY_WIFI_SSID"), password=os.getenv("CIRCUITPY_WIFI_PASSWORD"))
# print("my IP addr:", wifi.radio.ipv4_address)

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

for _ in range(10):
    led.value = True
    time.sleep(0.01)
    led.value = False
    time.sleep(0.01)

# create a PWMOut object on Pin A2.
pwm0 = pwmio.PWMOut(board.GP2, duty_cycle=2 ** 15, frequency=400)
pwm1 = pwmio.PWMOut(board.GP3, duty_cycle=2 ** 15, frequency=400)

# Create a servo object, left_servo.
left_servo = servo.Servo(pwm0)
right_servo = servo.Servo(pwm1)

print("initialize")
left_servo.angle = 0
right_servo.angle = 0

time.sleep(3)

print("1")
left_servo.angle = 0
right_servo.angle = 0

time.sleep(0.01)

print("2")
left_servo.angle = 90
right_servo.angle = 90

time.sleep(1)

increment = 0.5
delay = 0.0025
neutral_val = 90
spread = 60
max_val = neutral_val + spread
min_val = neutral_val - spread

while True:
    for angle in arange(neutral_val, max_val, increment):
        left_servo.angle = angle
        right_servo.angle = angle
        time.sleep(delay)
    for angle in arange(max_val, min_val, -increment):
        left_servo.angle = angle
        right_servo.angle = angle
        time.sleep(delay)
    for angle in arange(min_val, neutral_val, increment):
        left_servo.angle = angle
        right_servo.angle = angle
        time.sleep(delay)

