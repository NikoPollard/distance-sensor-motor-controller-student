import time
import board
import pwmio
import io
from adafruit_motor import motor

import ew_distance as ew_dist
ew_dist.setup()

import ew_uart as ua
import ew_imu
imu = ew_imu.IMU()
ua.setup("Niko UART")

PWM_PIN_A1 = board.D10  # pick any pwm pins on their own channels
PWM_PIN_A2 = board.D9
PWM_PIN_B1 = board.D7  # pick any pwm pins on their own channels
PWM_PIN_B2 = board.D8

DISTANCE_THROTTLES = {
    0:0,
    1:0,
    2:0.5,
    3:0.6,
    4:0.7,
    5:0.8,
    6:0.9,
    7:1,
    8:1,
    9:1,
    10:1,
    11:1,
    12:1,
    13:1,
    14:1,
    15:1,
    16:1,
}

# DC motor setup
pwm_a1 = pwmio.PWMOut(PWM_PIN_A1, frequency=50)
pwm_a2 = pwmio.PWMOut(PWM_PIN_A2, frequency=50)
motor1 = motor.DCMotor(pwm_a1, pwm_a2)

pwm_b1 = pwmio.PWMOut(PWM_PIN_B1, frequency=50)
pwm_b2 = pwmio.PWMOut(PWM_PIN_B2, frequency=50)
motor2 = motor.DCMotor(pwm_b1, pwm_b2)


motor1.throttle = 0
motor2.throttle = 0

print("-- throttle 1:", motor1.throttle)
print("-- throttle 2:", motor2.throttle)

MAX_RPM = 145  # Motor's max RPM at 100% duty cycle
stop_time = None
reverse_motors = False

# Function to get estimated RPM
def get_estimated_rpm(duty_cycle):
    return (duty_cycle/65535)*MAX_RPM # From Challenge #1

def get_all_rpm():
    return [
        get_estimated_rpm(pwm_a1.duty_cycle),
        get_estimated_rpm(pwm_a2.duty_cycle),
    ]

def write_rpms(rpms):
    msg = f"a1: {rpms[0]:.2f}; "
    msg += f"a2: {rpms[1]:.2f}\n"
    ua.write(msg)

# This function returns True if 5 seconds have passed since the vehicle stopped; otherwise, it returns False.

# Notes:
# - Make sure stop_time is not None before checking the elapsed time.
# - Use time.monotonic() to calculate how much time has passed
def check_reverse():
    global stop_time
    return stop_time != None and time.monotonic()-stop_time > 5

# This function checks the motor speeds and updates stop_time accordingly.

# If stop_time is None and both throttles are zero, then stop_time should be
# set to time.monotonic() — this marks the moment the motors stop.
# If one or both throttles are greater than zero, then stop_time should be reset to None.

# Note:
#  When modifying a variable declared outside the function (like stop_time),
#  you need to specify that it is global by using the 'global' keyword inside the function.
def check_stop():
    global stop_time
    if stop_time is None and motor1.throttle == 0 and motor2.throttle == 0:
        stop_time = time.monotonic()
    elif motor1.throttle > 0 or motor2.throttle > 0:
        stop_time = None

# This function sets the throttle on each motor. This is different from Challenge 1.
# In Challenge 1, the parameters represented how much to increase or decrease each throttle.
# That strategy turned out not to be a good one. Now, this function takes in the actual
# throttle values you want to set for each motor.

# You need to make sure that both m1_throt and m2_throt are between -1 and 1
# before setting the motor throttles. And the function should call check_stop() every
# time it is called

# Note:
# - Since the motors are mounted in opposite directions, they won't spin the same way
#   unless one throttle is negative and the other is positive.
# - However, this should be handled *inside* the method.
#   In other words, m1_throt and m2_throt should both be positive when moving forward,
#   and both negative when moving backward.
def set_throttle(m1_throt, m2_throt):
    if -1 <= m1_throt <= 1 and -1 <= m2_throt <= 1:
        motor1.throttle = m1_throt 
        motor2.throttle = m2_throt
    check_stop()

# This function moves the vehicle in reverse and simulates a turn. 
# It should be called when the vehicle has been blocked (not moving) for more than five seconds.

# Behavior:
# 1. Drive in full reverse until the distance sensor reads a value greater than 40.
# 2. Then, simulate a turn for 4 seconds by setting one motor to full power and the other to zero.
def maneuver_away_from_barrier():
    set_throttle(-1, -1)
    dist = ew_dist.read_distance()
    if dist == None: dist = 0
    while dist < 40:
        dist = ew_dist.read_distance()
        if dist == None: dist = 0
    elapsed = time.monotonic()
    while time.monotonic() - elapsed < 4:
        set_throttle(-1, 1)

counter = 0
while True:
    ua.connect()
    lastTime = time.monotonic_ns()
    while ua.connected():
        dt = time.monotonic_ns()-lastTime
        lastTime = time.monotonic_ns()
        imu.update(dt)
        print(imu.get_position(), imu.get_velocity(), imu.get_acceleration())
        print(imu.get_orientation(), imu.get_rot_velocity(), imu.get_rot_acceleration())
        counter += 1

        if counter % 500 == 0:
            write_rpms(get_all_rpm())
                
        if ua.in_waiting():
            data = ua.read(ua.in_waiting())
            if data:
                text = data.decode("utf-8").strip()
                print("Text Sent: ", text)
                if text and text == "w":
                    set_throttle(1, 1)
                elif text and text == "s":
                    set_throttle(0.1, 0.1)                
                elif text and text == "a":
                    set_throttle(1, 0.0)
                elif text and text == "d":
                    set_throttle(0, 1)
                write_rpms(get_all_rpm())
        dist = ew_dist.read_distance()
        if dist:
            # Write the distance to BlueConnect and probably print statement for now
            msg = f"Distance: {dist} cm"
            if counter % 100 == 0:
                ua.write(msg)
            print(msg)
            # set the distance at which you should go at an
            # appropriate throttle
            throttle = DISTANCE_THROTTLES[dist//10]
            set_throttle(throttle, throttle)
            if throttle == 0:
                if check_reverse():
                    maneuver_away_from_barrier()
                    
    set_throttle(0,0)
        
