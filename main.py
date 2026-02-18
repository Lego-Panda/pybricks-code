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

selected = hub_menu("1", "2", "3", "4", "5", "6", "7", "8", "9")

if selected == "1":
    run1()
if selected == "2":
    rob.shellButton(degrees=-365)
