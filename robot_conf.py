from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu, Matrix

hub = PrimeHub(Axis.Y, Axis.Z)

COLOR_PRESETS = {
    "stopYellow": (Color.YELLOW, Color.GREEN),
    "stopRed":    (Color.RED, Color.BLUE),
    "stopBlue":   (Color.BLUE, Color.YELLOW),
    "stopGreen":  (Color.GREEN, Color.RED)
}

leftwheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)
rightwheel = Motor(Port.B, Direction.CLOCKWISE)

shell = Motor(Port.E)
arm = Motor(Port.C)
colorS = ColorSensor(Port.D)

wheels = DriveBase(leftwheel, rightwheel, 62.4, 100)

CIRCUMFERENCE = 19.6
SHELL_RATIO = 360 / 1795
