import os
import time

import board
import digitalio
import pulseio
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


def get_sticks():
    channel1 = pulseio.PulseIn(board.GP14, maxlen=2, idle_state=False)
    channel1.resume(1)
    time.sleep(0.04)
    freq1, cycle1 = read_pwm(channel1)
    channel1.deinit()
    channel2 = pulseio.PulseIn(board.GP15, maxlen=2, idle_state=False)
    channel2.resume(1)
    time.sleep(0.04)
    freq2, cycle2 = read_pwm(channel2)
    channel2.deinit()

    if freq1 is None:
        cycle1 = None
    if freq2 is None:
        cycle2 = None
    if cycle1 == 0:
        cycle1 = None
    if cycle2 == 0:
        cycle2 = None
    return cycle1, cycle2


def read_pwm(pwm_in):
    if len(pwm_in) < 2:
        return None, 0
    pwm_in.pause()
    high_time = pwm_in[0]
    low_time = pwm_in[1]
    period = high_time + low_time
    # print(high_time,low_time, period)
    frequency = 1 / (period * 1e-6)  # Convert microseconds to seconds and then to frequency
    duty_cycle = high_time / (period) * 100  # Calculate duty cycle percentage
    if duty_cycle < 50:
        return None, duty_cycle
    pwm_in.clear()
    return frequency, duty_cycle


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

pwm0 = pwmio.PWMOut(board.GP2, duty_cycle=2**15, frequency=400)
pwm1 = pwmio.PWMOut(board.GP3, duty_cycle=2**15, frequency=400)

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

increment = 1.5
delay = 0.025
neutral_val = 90
spread = 60
max_val = neutral_val + spread
min_val = neutral_val - spread
max_cycle = 95.0652
min_cycle = 89.945


def scale_percent_to_angle(signed_percent):
    angle = (max_val - min_val) / 200 * (signed_percent + 100) + min_val
    return int(min(180, max(0, angle)))


def scale_cycle_to_percent(duty_cycle):
    percent = -200.0 / (max_cycle - min_cycle) * (duty_cycle - min_cycle) + 100.0
    return min(100.0, max(-100.0, percent))


up_down = 0
left_right = 0

max_up_down = None
max_left_right = None
min_up_down = None
min_left_right = None
while True:
    next_left_right, next_up_down = get_sticks()
    if next_up_down is not None:
        up_down = next_up_down
        max_up_down = max(max_up_down, up_down) if max_up_down is not None else up_down
        max_left_right = max(max_left_right, left_right) if max_left_right is not None else left_right
    if next_left_right is not None:
        left_right = next_left_right
        min_up_down = min(min_up_down, up_down) if min_up_down is not None else up_down
        min_left_right = min(min_left_right, left_right) if min_left_right is not None else left_right

    # print("\t".join([str(x) for x in (max_up_down, max_left_right, min_up_down, min_left_right)]))
    up_down_percent = -1 * scale_cycle_to_percent(up_down)
    left_right_percent = scale_cycle_to_percent(left_right)

    left = up_down_percent - left_right_percent
    right = up_down_percent + left_right_percent
    left_servo.angle = scale_percent_to_angle(left)
    right_servo.angle = scale_percent_to_angle(right)

    # for angle in arange(neutral_val, max_val, increment):
    #     left_servo.angle = angle
    #     right_servo.angle = angle
    #     time.sleep(delay)
    #     print_analog()
    # for angle in arange(max_val, min_val, -increment):
    #     left_servo.angle = angle
    #     right_servo.angle = angle
    #     time.sleep(delay)
    #     print_analog()
    # for angle in arange(min_val, neutral_val, increment):
    #     left_servo.angle = angle
    #     right_servo.angle = angle
    #     time.sleep(delay)
    #     print_analog()
