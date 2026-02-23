from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import * 
from functions import *

hub = PrimeHub(Axis.Y, Axis.Z)

def run1():
    rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    rob.pid(65, -525)
    wait(300)
    rob.turn(-30, 125)
    wait(300)
    arm.run_time(-660, 900)
    wait(300)
    rob.pid(3, -200)

    # rob.pid(10000000, 1000)

# run1()

def run2():
    rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)
    
    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    rob.pid(42, -525)
    for i in range(4):
        arm.run_time(-700, 700)
        wait(200)
        arm.run_time(700, 700)
    wait(200)
    rob.pid(50, 525)

run2()

def run5():
    rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)


    rob.pid(70, -525)
    # rob.AccelDecel(70, -525)
    wait(300)

run5()

def battery():
    rob = Robot()

    rob.battery()

# battery()
