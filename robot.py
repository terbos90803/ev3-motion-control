#!/usr/bin/env python3
"""
SSCI SoccerBot robot server
"""

import remote
import pickle
import json
import time

from ev3dev2 import DeviceNotFound
from ev3dev2.motor import MediumMotor, MoveSteering, SpeedPercent, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, OUTPUT_D
from ev3dev2.led import Led, Leds
from ev3dev2.display import Display
from Screen import init_console, reset_console, debug_print


FORWARD = u'\u2191' # up-arrow glyph
REVERSE = u'\u2193' # down-arrow glyph
LEFT = u'\u2190' # left-arrow glyph
RIGHT = u'\u2192' # right-arrow glyph
GRAB = 'Grab'
RELEASE = 'Release'

driveSpeed = 35
turnSpeed = 35
liftSpeed = 40
holdSpeed = 10

init_console()

# get handles for the three motors
liftMotor = None
steeringDrive = None
spinMotor = None
while True:
    try:
        liftMotor = liftMotor if liftMotor else MediumMotor(OUTPUT_A)
        steeringDrive = steeringDrive if steeringDrive else MoveSteering(OUTPUT_B, OUTPUT_C)
        spinMotor = spinMotor if spinMotor else MediumMotor(OUTPUT_D)
        break
    except DeviceNotFound as error:
        print("Motor not connected")
        print("Check and restart")
        print(error)
        time.sleep(1)

debug_print('stop actions:', liftMotor.stop_actions)
debug_print('pickle.HIGHEST_PROTOCOL:', pickle.HIGHEST_PROTOCOL)


def move(value):
    steeringDrive.on_for_rotations(0, driveSpeed, 1.0 * value)


def turn(value):
    steeringDrive.on_for_rotations(100, turnSpeed, 0.4925 * value)


def lift_up():
    liftMotor.duty_cycle_sp = -liftSpeed
    liftMotor.run_direct()
    time.sleep(0.1)
    liftMotor.wait_until_not_moving()
    liftMotor.duty_cycle_sp = -holdSpeed


def lift_down():
    liftMotor.stop_action = 'coast'
    liftMotor.duty_cycle_sp = liftSpeed
    liftMotor.run_direct()
    time.sleep(0.1)
    liftMotor.wait_until_not_moving()
    liftMotor.stop()


def ramp_motor(sp):
    spinMotor.duty_cycle_sp = sp
    t = time.time()
    at = t
    ma = 0
    a0 = a1 = 0.0
    last_ms = time.time() * 1000.0
    s = spinMotor.speed
    while time.time() - t < 1.0:
        # when is now?
        now_ms = time.time() * 1000.0
        # don't sample more frequently than 1000Hz
        while now_ms - last_ms < 1.0:
            now_ms = time.time() * 1000.0
        # how long since the last sample?
        dt = now_ms - last_ms
        last_ms = now_ms
        # how fast is the motor going?
        cs = spinMotor.speed
        # acceleration = change in velocity over time
        a = (cs - s) / dt
        s = cs
        # median filter the acceleration
        # 3 samples are in age order: a, a0, a1
        # step 1: sort two into increasing order
        if a0 < a1:
            al = a0
            ah = a1
        else:
            al = a1
            ah = a0
        # bubble the third into the set and pick the middle
        af = max(a, al)
        af = min(af, ah)
        # age the sample history
        a1 = a0
        a0 = a
        # capture the max accel so far
        ma = max(ma, abs(af))
        # note the time when the velocity has stabilized
        if abs(af) > 1.0:
            at = time.time()
    debug_print('speed: {:4}   accel time: {:.1f}   max accel: {:.1f}'.format(s, (at-t)*1000.0, ma))


def spin_test():
    spinMotor.run_direct()
    while True:
        ramp_motor(100)
        ramp_motor(50)


leds = Leds()
#debug_print(Led().triggers)
leds.set('LEFT', trigger='default-on')
leds.set('RIGHT', trigger='default-on')

display = Display()
screenw = display.xres
screenh = display.yres

# reset the lift motor to a known good position
lift_down()

# run motor test
spin_test()

# Create connection to server
s, host_address = remote.get_listener_socket()
s.settimeout(5)

# Main loop handles connections to the host
while True:
    try:
        reset_console()
        print (host_address)
        leds.set_color('LEFT', 'AMBER')
        leds.set_color('RIGHT', 'AMBER')
        debug_print('Waiting for connection')
        remote.advertise()

        try:
            client, clientInfo = s.accept()
        except:
            continue

        client.settimeout(10)
        print ('Connected')
        debug_print('Connected to:', clientInfo)
        leds.set_color('LEFT', 'GREEN')
        leds.set_color('RIGHT', 'GREEN')

        # Driving loop
        while True:
            data = client.recv(remote.size)
            #debug_print('recv:', data)
            if data == b'':
                debug_print('Disconnecting because received empty packet')
                client.close()
                break

            if data:
                #debug_print(data)
                jd = pickle.loads(data)
                #debug_print(jd)
                if jd == 'ping':
                    continue

                sequence = json.loads(jd)

                # command format: [[cmd, value], ...]
                #debug_print(sequence)
                if isinstance(sequence, list):
                    for step in sequence:
                        #debug_print(step)
                        cmd = step[0]
                        #debug_print(cmd)
                        value = float(step[1]) if len(step) > 1 else 0.0
                        #debug_print(value)
                        if cmd == FORWARD:
                            move(value)
                        elif cmd == REVERSE:
                            move(-value)
                        elif cmd == LEFT:
                            turn(-value)
                        elif cmd == RIGHT:
                            turn(value)
                        elif cmd == GRAB:
                            grab()
                        elif cmd == RELEASE:
                            release()
    except Exception as e:
        debug_print('Disconnecting because exception:', e)
        client.close()
s.close()
