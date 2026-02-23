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
    rob.turn(-25, 125)
    wait(300)
    arm.run(-700)
    wait(400)
    arm.stop()
    wait(300)
    rob.pid(3, -100)
    wait(300)
    arm.run_time(660, 3000)
    wait(300)
    rob.pid(6, 100)
    wait(300)
    rob.turn(20, 125)
    wait(300)
    rob.pid(6, -100)
    wait(300)
    rob.shellTurn(70, 400)
    rob.turn(70, 125)

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

# run2()

def run3():
    rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    rob.pid(40, -525)
    wait(100)
    arm.run(-1000)
    wait(1000)
    arm.stop()
    wait(100)
    rob.pid(13, 525)
    wait(100)
    arm.run(1000)
    wait(1300)
    arm.stop()
    wait(100)
    rob.pid(11, -525)
    wait(100)
    rob.shellTurn(150, 800)
    wait(100)
    rob.pid(30, 525)
    wait(100)
    arm.run(700)
    wait(1300)
    arm.stop()
    wait(100)
    rob.stopColor("stopYellow")

# run3()

def run7():
    rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    rob.pid(50,-500)
    wait(400)
    rob.pid(3,500)
    wait(400)
    arm.run_time(660, 1000)
    wait(400)
    rob.pid(4,650)
    wait(200)
    rob.pid(5, -650)
    wait(200)
    # rob.pid(15,525)
    # wait(200)
    rob.pid(45, 525)

def battery():
    rob = Robot()

    rob.battery()

# battery()
