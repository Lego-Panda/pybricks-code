from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import * 
from functions import *
from runs import *

hub = PrimeHub(Axis.Y, Axis.Z)

rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)

hub.display.orientation(Side.BOTTOM)
selected = hub_menu("H","1", "2", "3", "4", "5", "6", "7", "8", "B")

if selected == "H":
    leftwheel.run(1000)
    rightwheel.run(-1000)
    wait(100000000)
if selected == "1":
    run1()
if selected == "2":
    run2()
if selected == "3":
    run3()
if selected == "4":
    run4()
if selected == "5":
    run5()
if selected == "6":
    pass
if selected == "7":
    run7()
if selected == "8":
    pass
