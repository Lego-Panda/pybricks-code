from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu

hub = PrimeHub(Axis.Y, Axis.Z) # SPIKE Prime Hub

WHEEL_CIRCUMFEFRENCE = 196
SHELL_RATIO = 360 / 1795

leftwheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)
rightwheel = Motor(Port.B, Direction.CLOCKWISE)

shell = Motor(Port.E)
arm = Motor(Port.C)
colorSens = ColorSensor(Port.D)

wheels = DriveBase(leftwheel, rightwheel, 62.4, 100)

gyro = hub.imu
