from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import * 
from functions import *

hub = PrimeHub(Axis.Y, Axis.Z)
hub.display.orientation(Side.BOTTOM)

def run1():
    rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    rob.decel(70, -525)
    wait(300)
    rob.turn(-20, 125)
    wait(300)
    arm.run(-700)
    wait(400)
    arm.stop()
    wait(600)
    rob.turn(-20, 200)
    wait(300)
    rob.pid(3, -100)
    wait(300)
    arm.run_time(660, 1000)
    wait(500)
    rob.pid(6, 100)
    wait(300)
    rob.turn(20, 125)
    wait(300)
    rob.pid(5, -100)
    wait(300)
    shell.run_time(1000, 600)
    wait(300)
    rob.pid(10, 200)
    wait(300)
    rob.turn(50, 125)
    wait(300)
    rob.pid(60, 525)
    wait(300)
    rob.shellButton(-180)

####

def run2():
    rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    # rob.pid(44, -525)
    rob.decel(46, -525)
    wait(200)
    for i in range(4):
        arm.run_time(-700, 700)
        wait(200)
        arm.run_time(700, 700)
    wait(200)
    rob.pid(50, 525)

####

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

####
def run4():
    rob = Robot(kp=5, ki=0, kd=10, turnKp=1, turnKi=0, turnKd=0,shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # H igh G
    wait(400)

    rob.accelDecel(22, 525)
    wait(300)
    rob.turn(90, 200)
    wait(300)
    rob.accelDecel(62, 525)
    wait(300)
    rob.turnWhileShell(-90, 90, 1000, 125)
    wait(300)
    rob.pid(8, 150)
    wait(300)
    rob.shellTurn(-10, -400)
    wait(300)
    arm.run_time(-660, 3000)
    wait(300)
    rob.pid(10,-300)
    wait(400)
    rob.turn(-55, 200)
    wait(300)
    rob.pid(13, -100)
    wait(200)
    rob.turn(-20, 200)
    # wait(200)
    # rob.accelDecel(70, 650)
    # wait(200)
    # rob.turn(30, 200)
    # wait(300)
    # rob.accelDecel(20, 650)
    # wait(300)
    # rob.stopColor("stopYellow")

###

def run5():
    rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)

    arm.run_time(1000, 600)
    wait(300)
    rob.accelDecel(69, -650)
    wait(300)
    rob.turn(45, 200)
    wait(300)
    leftwheel.run(1000)
    rightwheel.run(-1000)
    wait(3000)
    leftwheel.stop()
    rightwheel.stop()
    wait(300)
    arm.run_time(1000, 1000)
    wait(300)
    rob.pid(15, 525)

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

def test():
    # rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)
    rob = Robot(kp=5, ki=0, kd=10, turnKp=1, turnKi=0, turnKd=0,shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    # rob.turnWhileShell(90, -90, 600, 125)
    arm.run_time(1000, 10000)

###

def battery():
    rob = Robot()
    # rob.battery_percent()
    hub.display.icon(Icon.HAPPY)

######

# run1()

# run2()

# run3()

run4()

# run5()

# run7()

# test()

# battery()
