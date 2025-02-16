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

left_servo.angle = 0
right_servo.angle = 0

time.sleep(0.01)

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

while True:
    next_left_right, next_up_down = get_sticks()
    if next_up_down is not None:
        up_down = next_up_down
    if next_left_right is not None:
        left_right = next_left_right

    up_down_percent = -1 * scale_cycle_to_percent(up_down)
    left_right_percent = scale_cycle_to_percent(left_right)

    print(f"{up_down_percent}\t{left_right_percent}")

    left = up_down_percent - left_right_percent
    right = up_down_percent + left_right_percent
    left_servo.angle = scale_percent_to_angle(left)
    right_servo.angle = scale_percent_to_angle(right)
